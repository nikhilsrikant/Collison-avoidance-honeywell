#!/usr/bin/env python3
"""Runway station: the Waymo-style loop (perceive -> track -> predict -> decide) on a tabletop runway.

A camera looks down at a printed runway. The script detects aircraft and vehicles, tracks them,
predicts where each will be in the next few seconds, and raises a RUNWAY INCURSION when a vehicle or
person is on, or about to enter, a runway that is in use. Nothing has to be installed in the
aircraft, and the vehicle does not need a transponder (the LaGuardia fire truck had none).

Outputs
  * on-screen overlay (and an annotated video with --out)
  * Arduino lights and buzzer over USB serial (runway entrance lights: red = do not enter)
  * status to server.py so the 3D display can tell its airplane to go around

Quick start
  python runway_guard.py --detector color          # colored tape on the toys, most reliable
  python runway_guard.py --detector yolo           # pretrained YOLO (pip install ultralytics)

Keys: f = toggle "aircraft on final", c = recalibrate runway, v / a = click to sample vehicle / aircraft
color, space = pause, s = snapshot, q or Esc = quit.
"""
import argparse
import csv
import json
import math
import os
import queue
import sys
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(HERE, "runway_calib.json")
LOG_FILE = os.path.join(HERE, "runway_log.csv")

# COCO class -> our class. Toy airplanes are often read as "bird" or "kite", so those count as aircraft here.
CLASS_MAP = {
    "airplane": "aircraft", "bird": "aircraft", "kite": "aircraft",
    "car": "vehicle", "truck": "vehicle", "bus": "vehicle", "motorcycle": "vehicle", "bicycle": "vehicle", "train": "vehicle",
    "person": "person",
}
COLORS = {"aircraft": (255, 200, 60), "vehicle": (40, 60, 255), "person": (0, 200, 255), "object": (200, 200, 200)}
STATE_COLORS = {"CLEAR": (90, 200, 90), "ACTIVE": (0, 170, 255), "OCCUPIED": (0, 200, 255), "INCURSION": (40, 40, 255), "OFFLINE": (120, 120, 120)}
SERIAL_CODES = {"CLEAR": "C", "ACTIVE": "A", "OCCUPIED": "O", "INCURSION": "I"}


# ----------------------------------------------------------------------------- detectors
class ColorDetector:
    """Finds colored markers: red/orange tape = vehicle, blue tape = aircraft. Press v or a and click to re-sample."""

    def __init__(self, min_area=250):
        self.min_area = min_area
        self.ranges = {
            "vehicle": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
            "aircraft": [((95, 120, 60), (130, 255, 255))],
        }

    def sample(self, frame, x, y, cls):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        patch = hsv[max(0, y - 4):y + 5, max(0, x - 4):x + 5].reshape(-1, 3).astype(int)
        h, s, v = np.median(patch, axis=0)
        lo = (max(0, int(h) - 10), max(40, int(s) - 70), max(40, int(v) - 70))
        hi = (min(180, int(h) + 10), 255, 255)
        self.ranges[cls] = [(lo, hi)]
        print("[color] %s now H %d-%d, S >= %d, V >= %d" % (cls, lo[0], hi[0], lo[1], lo[2]))

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        out = []
        kernel = np.ones((5, 5), np.uint8)
        for cls, rngs in self.ranges.items():
            mask = None
            for lo, hi in rngs:
                m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                out.append((cls, 0.9, (x, y, x + w, y + h)))
        return out


class YoloDetector:
    """Pretrained YOLO (COCO classes). Downloads yolov8n.pt the first time (about 6 MB)."""

    def __init__(self, model="yolov8n.pt", conf=0.25, imgsz=640):
        from ultralytics import YOLO  # pip install ultralytics
        self.model = YOLO(model)
        self.conf, self.imgsz = conf, imgsz
        self.names = self.model.names

    def detect(self, frame):
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        out = []
        for b in res.boxes:
            name = self.names[int(b.cls[0])]
            cls = CLASS_MAP.get(name)
            if cls is None:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            out.append((cls, float(b.conf[0]), (x1, y1, x2, y2)))
        return out


class MotionDetector:
    """Anything that moves. Every blob is an unknown 'object' and is treated like a vehicle."""

    def __init__(self, min_area=400):
        self.bg = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=32, detectShadows=False)
        self.min_area = min_area

    def detect(self, frame):
        mask = self.bg.apply(frame)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.dilate(mask, np.ones((9, 9), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            if cv2.contourArea(c) >= self.min_area:
                x, y, w, h = cv2.boundingRect(c)
                out.append(("object", 0.5, (x, y, x + w, y + h)))
        return out


# ----------------------------------------------------------------------------- tracking + prediction
class Track:
    def __init__(self, tid, cls, box, t):
        self.id = tid
        self.votes = {cls: 1}
        self.box = box
        self.c = ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)
        self.v = (0.0, 0.0)
        self.last = t
        self.hits = 1
        self.alert_t = None     # first time we predicted it would enter the active runway
        self.entered_t = None   # first time it was actually on the active runway
        self.lead = None        # seconds of warning we gave before it entered

    @property
    def cls(self):
        return max(self.votes, key=self.votes.get)

    def update(self, cls, box, t):
        self.votes[cls] = self.votes.get(cls, 0) + 1
        c = ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)
        dt = t - self.last
        if dt > 1e-3:
            vx, vy = (c[0] - self.c[0]) / dt, (c[1] - self.c[1]) / dt
            a = 0.45
            self.v = (self.v[0] * (1 - a) + vx * a, self.v[1] * (1 - a) + vy * a)
        self.c, self.box, self.last = c, box, t
        self.hits += 1

    def speed(self):
        return math.hypot(*self.v)

    def predict(self, dt):
        return (self.c[0] + self.v[0] * dt, self.c[1] + self.v[1] * dt)


class Tracker:
    def __init__(self, max_dist=110, max_age=0.8):
        self.tracks = {}
        self.next_id = 1
        self.max_dist, self.max_age = max_dist, max_age

    def step(self, dets, t):
        unmatched = list(range(len(dets)))
        pairs = []
        for tid, tr in self.tracks.items():
            pc = tr.predict(t - tr.last)
            for i in unmatched:
                cls, _, b = dets[i]
                c = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)
                d = math.hypot(c[0] - pc[0], c[1] - pc[1]) + (0 if cls == tr.cls else 40)
                if d < self.max_dist:
                    pairs.append((d, tid, i))
        pairs.sort()
        used_t, used_d = set(), set()
        for d, tid, i in pairs:
            if tid in used_t or i in used_d:
                continue
            used_t.add(tid)
            used_d.add(i)
            cls, _, b = dets[i]
            self.tracks[tid].update(cls, b, t)
        for i in range(len(dets)):
            if i not in used_d:
                cls, _, b = dets[i]
                self.tracks[self.next_id] = Track(self.next_id, cls, b, t)
                self.next_id += 1
        for tid in [k for k, tr in self.tracks.items() if t - tr.last > self.max_age]:
            del self.tracks[tid]
        return [tr for tr in self.tracks.values() if tr.hits >= 3]


# ----------------------------------------------------------------------------- runway geometry
class Runway:
    """Four clicked corners: the two at the landing end first, then the far end."""

    def __init__(self, pts):
        self.pts = np.array(pts, dtype=np.float32)
        p0, p1, p2, p3 = self.pts
        mid_land, mid_far = (p0 + p1) / 2, (p2 + p3) / 2
        axis = mid_land - mid_far
        length = float(np.linalg.norm(axis)) or 1.0
        u = axis / length
        self.width = float(np.linalg.norm(p1 - p0))
        self.length = length
        # final approach zone: beyond the landing end, 60 percent of the runway length
        self.approach = np.array([p0, p1, p1 + u * length * 0.6, p0 + u * length * 0.6], dtype=np.float32)
        self.hold_margin = max(14.0, 0.45 * self.width)

    def dist(self, p):
        """Signed distance in pixels: positive inside the runway, negative outside."""
        return cv2.pointPolygonTest(self.pts.reshape(-1, 1, 2), (float(p[0]), float(p[1])), True)

    def in_approach(self, p):
        return cv2.pointPolygonTest(self.approach.reshape(-1, 1, 2), (float(p[0]), float(p[1])), False) >= 0

    def to_json(self, w, h):
        return {"pts": self.pts.tolist(), "w": w, "h": h}

    @staticmethod
    def from_json(d, w, h):
        sx, sy = w / float(d["w"]), h / float(d["h"])
        return Runway([(x * sx, y * sy) for x, y in d["pts"]])


def calibrate(cap, w, h):
    pts = []
    win = "Calibrate runway"

    def on_mouse(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    msg = ["Click the 4 runway corners:", "1-2 = landing end (where planes touch down), 3-4 = far end.", "Enter = save, r = redo, q = quit"]
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        disp = frame.copy()
        for i, t in enumerate(msg):
            cv2.putText(disp, t, (14, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
            cv2.putText(disp, t, (14, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        for i, p in enumerate(pts):
            cv2.circle(disp, p, 7, (0, 255, 255), -1)
            cv2.putText(disp, str(i + 1), (p[0] + 9, p[1] - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if len(pts) >= 2:
            cv2.polylines(disp, [np.array(pts, np.int32)], len(pts) == 4, (0, 255, 255), 2)
        cv2.imshow(win, disp)
        k = cv2.waitKey(20) & 0xFF
        if k in (13, 10) and len(pts) == 4:
            break
        if k == ord("r"):
            pts.clear()
        if k in (ord("q"), 27):
            sys.exit(0)
    cv2.destroyWindow(win)
    rw = Runway(pts)
    with open(CALIB_FILE, "w") as f:
        json.dump(rw.to_json(w, h), f)
    print("[calib] saved", CALIB_FILE)
    return rw


# ----------------------------------------------------------------------------- outputs
class SerialLink:
    """Arduino on USB serial: we send the state letter, it sends back 'D <cm>' from the hold-line sensor."""

    def __init__(self, port):
        self.ser, self.dist_cm, self.dist_t = None, None, 0.0
        if port in (None, "none"):
            return
        try:
            import serial
            import serial.tools.list_ports
        except ImportError:
            print("[serial] pyserial not installed (pip install pyserial); running without Arduino")
            return
        if port == "auto":
            ports = list(serial.tools.list_ports.comports())
            cands = [p.device for p in ports
                     if any(k in (p.description or "") + (p.manufacturer or "") for k in ("Arduino", "CH340", "CP210", "USB Serial", "USB-SERIAL"))]
            cands += [p.device for p in ports if any(k in p.device for k in ("usbmodem", "usbserial", "wchusbserial", "ttyACM", "ttyUSB")) and p.device not in cands]
            port = cands[0] if cands else None
            if not port:
                print("[serial] no Arduino found; running without lights (use --serial COM3 or /dev/ttyACM0)")
                return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0)
            time.sleep(2.0)  # the Arduino resets when the port opens
            print("[serial] connected on", port)
        except Exception as e:
            print("[serial] could not open %s: %s" % (port, e))
            self.ser = None
        self.buf = b""

    def send(self, state):
        if not self.ser:
            return
        try:
            self.ser.write((SERIAL_CODES.get(state, "C") + "\n").encode())
        except Exception as e:
            print("[serial] write failed:", e)
            self.ser = None

    def poll(self):
        if not self.ser:
            return
        try:
            self.buf += self.ser.read(256)
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                line = line.decode(errors="ignore").strip()
                if line.startswith("D "):
                    try:
                        v = float(line[2:])
                        self.dist_cm = v if v > 0 else None
                        self.dist_t = time.time()
                    except ValueError:
                        pass
        except Exception:
            pass


class ServerLink(threading.Thread):
    """Posts our state to server.py and reads whether the 3D display's airplane is on final."""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url.rstrip("/") if url and url != "none" else None
        self.q = queue.Queue(maxsize=4)
        self.display = None
        self.ok = False
        if self.url:
            self.start()

    def post(self, payload):
        if self.url and not self.q.full():
            self.q.put(payload)

    def run(self):
        last_get = 0
        while True:
            try:
                payload = self.q.get(timeout=0.5)
                req = urllib.request.Request(self.url + "/api/runway", data=json.dumps(payload).encode(),
                                             headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=1.5).read()
                self.ok = True
            except queue.Empty:
                pass
            except Exception:
                self.ok = False
            if time.time() - last_get > 0.5:
                last_get = time.time()
                try:
                    with urllib.request.urlopen(self.url + "/api/state", timeout=1.5) as r:
                        self.display = json.loads(r.read().decode())
                    self.ok = True
                except Exception:
                    self.display = None


class Voice(threading.Thread):
    def __init__(self, enabled):
        super().__init__(daemon=True)
        self.q = queue.Queue(maxsize=2)
        self.engine = None
        if enabled:
            try:
                import pyttsx3
                self.engine = pyttsx3.init()
                self.start()
            except Exception as e:
                print("[voice] off:", e)

    def say(self, text):
        if self.engine and not self.q.full():
            self.q.put(text)

    def run(self):
        while True:
            t = self.q.get()
            try:
                self.engine.say(t)
                self.engine.runAndWait()
            except Exception:
                pass


# ----------------------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="webcam index (try 1 for an external or phone camera)")
    ap.add_argument("--video", help="use a video file instead of a camera")
    ap.add_argument("--detector", choices=["auto", "color", "yolo", "motion"], default="auto")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--serial", default="auto", help="auto, none, or a port like COM3 or /dev/ttyACM0")
    ap.add_argument("--server", default="http://localhost:8765", help="server.py address, or none")
    ap.add_argument("--runway", help='corners as "x1,y1;x2,y2;x3,y3;x4,y4" (landing end first)')
    ap.add_argument("--recalibrate", action="store_true")
    ap.add_argument("--predict", type=float, default=2.5, help="seconds to look ahead")
    ap.add_argument("--min-speed", type=float, default=12.0, help="pixels per second before we predict motion")
    ap.add_argument("--sensor-cm", type=float, default=12.0, help="hold-line sensor distance that counts as a vehicle")
    ap.add_argument("--voice", action="store_true", help="spoken alerts on this laptop (pip install pyttsx3)")
    ap.add_argument("--headless", action="store_true", help="no windows (for testing)")
    ap.add_argument("--out", help="write an annotated video, e.g. out.mp4")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        cap = cv2.VideoCapture(args.camera, backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    ok, frame = cap.read()
    if not ok:
        print("Could not read from the camera/video. Try --camera 1, close other apps using the camera, or check permissions.")
        sys.exit(1)
    H, W = frame.shape[:2]
    fps_src = cap.get(cv2.CAP_PROP_FPS) or 30.0

    # runway
    if args.runway:
        rw = Runway([tuple(map(float, p.split(","))) for p in args.runway.split(";")])
    elif os.path.exists(CALIB_FILE) and not args.recalibrate:
        with open(CALIB_FILE) as f:
            rw = Runway.from_json(json.load(f), W, H)
        print("[calib] loaded", CALIB_FILE, "(use --recalibrate or press c to redo)")
    elif args.headless:
        print("Headless mode needs --runway or a saved calibration.")
        sys.exit(1)
    else:
        rw = calibrate(cap, W, H)

    # detector
    det_name = args.detector
    if det_name == "auto":
        try:
            import ultralytics  # noqa: F401
            det_name = "yolo"
        except ImportError:
            det_name = "color"
    detector = {"color": ColorDetector, "motion": MotionDetector}.get(det_name, None)
    detector = detector() if detector else YoloDetector(args.model)
    print("[detector]", det_name)

    tracker = Tracker()
    link = SerialLink(args.serial)
    server = ServerLink(args.server)
    voice = Voice(args.voice)
    writer = None
    if args.out:
        writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps_src if args.video else 15, (W, H))

    log_new = not os.path.exists(LOG_FILE)
    logf = open(LOG_FILE, "a", newline="")
    log = csv.writer(logf)
    if log_new:
        log.writerow(["time", "state", "detail", "lead_s"])

    win = "Runway station"
    sample_mode = [None]
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        def on_mouse(ev, x, y, flags, param):
            if ev == cv2.EVENT_LBUTTONDOWN and sample_mode[0] and isinstance(detector, ColorDetector):
                detector.sample(param["frame"], x, y, sample_mode[0])
                sample_mode[0] = None

        mouse_param = {"frame": frame}
        cv2.setMouseCallback(win, on_mouse, mouse_param)

    manual_final = False
    paused = False
    state, prev_state = "CLEAR", None
    incursion_until = -1.0
    last_send = 0.0
    leads = deque(maxlen=5)
    frames = 0
    t0 = time.time()
    fps = 0.0
    fps_t, fps_n = time.time(), 0
    video_t = 0.0

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                if args.video:
                    break
                continue
            frames += 1
        if args.video:
            video_t = frames / fps_src
            now = video_t
        else:
            now = time.time() - t0

        dets = detector.detect(frame)
        tracks = tracker.step(dets, now)
        link.poll()

        # --- decide ---
        disp_final = bool(server.display and server.display.get("on_final") and (server.display.get("age") or 99) < 3)
        aircraft_active = any(tr.cls == "aircraft" and (rw.dist(tr.c) >= -rw.hold_margin or rw.in_approach(tr.c)) for tr in tracks)
        runway_active = aircraft_active or disp_final or manual_final
        sensor_hit = link.dist_cm is not None and link.dist_cm < args.sensor_cm and time.time() - link.dist_t < 1.0

        conflicts = []
        on_runway_vehicle = False
        for tr in tracks:
            if tr.cls == "aircraft":
                continue
            d = rw.dist(tr.c)
            tr.on_runway = d >= 0
            tr.t_entry = None
            if not tr.on_runway and tr.speed() > args.min_speed:
                step = 0.1
                k = 1
                while k * step <= args.predict:
                    if rw.dist(tr.predict(k * step)) >= 0:
                        tr.t_entry = k * step
                        break
                    k += 1
            if tr.on_runway:
                on_runway_vehicle = True
            if runway_active and (tr.on_runway or tr.t_entry is not None):
                conflicts.append(tr)
                if tr.alert_t is None:
                    tr.alert_t = now
            if runway_active and tr.on_runway and tr.entered_t is None:
                tr.entered_t = now
                if tr.alert_t is not None:
                    tr.lead = tr.entered_t - tr.alert_t
                    leads.append(tr.lead)
                    log.writerow([time.strftime("%H:%M:%S"), "ENTRY", "track %d entered" % tr.id, "%.2f" % tr.lead])

        if runway_active and (conflicts or sensor_hit):
            state = "INCURSION"
            incursion_until = now + 1.5          # latch so the alarm does not flicker while estimates settle
        elif runway_active and now < incursion_until:
            state = "INCURSION"
        elif on_runway_vehicle or sensor_hit:
            state = "OCCUPIED"
        elif runway_active:
            state = "ACTIVE"
        else:
            state = "CLEAR"

        if conflicts:
            c0 = conflicts[0]
            detail = "%s %d %s" % (c0.cls, c0.id, "on the runway" if c0.on_runway else "enters in %.1f s" % c0.t_entry)
        elif sensor_hit:
            detail = "hold-line sensor: object at %.0f cm" % link.dist_cm
        elif state == "INCURSION":
            detail = "incursion (holding alarm)"
        elif on_runway_vehicle:
            detail = "vehicle on the runway"
        elif runway_active:
            detail = "aircraft landing" + (" (3D display)" if disp_final and not aircraft_active else "") + (" (manual)" if manual_final else "")
        else:
            detail = "runway clear"

        if state != prev_state:
            log.writerow([time.strftime("%H:%M:%S"), state, detail, ""])
            logf.flush()
            print("[%6.1fs] %-9s %s" % (now, state, detail))
            if state == "INCURSION":
                voice.say("Stop. Runway incursion.")
            prev_state = state

        if time.time() - last_send > 0.25 or state == "INCURSION":
            last_send = time.time()
            link.send(state)
            server.post({"state": state, "detail": detail, "aircraft": sum(1 for t in tracks if t.cls == "aircraft"),
                         "vehicles": sum(1 for t in tracks if t.cls != "aircraft"), "sensor_cm": link.dist_cm,
                         "lead_s": leads[-1] if leads else None})

        # --- draw ---
        out = frame.copy()
        overlay = out.copy()
        col = STATE_COLORS[state]
        cv2.fillPoly(overlay, [rw.pts.astype(np.int32)], col)
        cv2.addWeighted(overlay, 0.25 if state != "INCURSION" or int(time.time() * 6) % 2 else 0.5, out, 0.75, 0, out)
        cv2.polylines(out, [rw.pts.astype(np.int32)], True, col, 2)
        cv2.polylines(out, [rw.approach.astype(np.int32)], True, (200, 200, 200), 1, cv2.LINE_AA)
        ax, ay = rw.approach[3]
        cv2.putText(out, "final approach", (int(ax) + 4, int(ay) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        for tr in tracks:
            c = COLORS.get(tr.cls, (255, 255, 255))
            x1, y1, x2, y2 = tr.box
            cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
            cv2.putText(out, "%s %d" % (tr.cls, tr.id), (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
            if tr.speed() > args.min_speed:
                for k in range(1, 6):
                    p = tr.predict(k * args.predict / 5)
                    cv2.circle(out, (int(p[0]), int(p[1])), 3, c, -1)
                p = tr.predict(1.0)
                cv2.arrowedLine(out, (int(tr.c[0]), int(tr.c[1])), (int(p[0]), int(p[1])), c, 2, tipLength=0.25)
            if getattr(tr, "t_entry", None) is not None:
                cv2.putText(out, "enters in %.1fs" % tr.t_entry, (x1, y2 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 40, 255), 2)
        # banner
        cv2.rectangle(out, (0, 0), (W, 44), (16, 16, 16), -1)
        label = {"CLEAR": "RUNWAY CLEAR", "ACTIVE": "RUNWAY IN USE: VEHICLES HOLD", "OCCUPIED": "RUNWAY OCCUPIED",
                 "INCURSION": "RUNWAY INCURSION: STOP"}[state]
        cv2.putText(out, label, (12, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
        cv2.putText(out, detail, (W // 2, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        fps_n += 1
        if time.time() - fps_t > 1:
            fps, fps_t, fps_n = fps_n / (time.time() - fps_t), time.time(), 0
        info = "%s | %.0f fps | sensor2: %s | server: %s | lead: %s" % (
            det_name, fps, ("%.0f cm" % link.dist_cm) if link.dist_cm is not None else "off",
            "linked" if server.ok else "off", ("%.1f s" % leads[-1]) if leads else "-")
        cv2.rectangle(out, (0, H - 28), (W, H), (16, 16, 16), -1)
        cv2.putText(out, info, (10, H - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        if manual_final:
            cv2.putText(out, "MANUAL: aircraft on final (f)", (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 170, 255), 2)
        if sample_mode[0]:
            cv2.putText(out, "Click the %s color" % sample_mode[0], (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if writer:
            writer.write(out)
        if not args.headless:
            mouse_param["frame"] = frame
            cv2.imshow(win, out)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("f"):
                manual_final = not manual_final
            if k == ord(" "):
                paused = not paused
            if k == ord("c"):
                rw = calibrate(cap, W, H)
            if k == ord("v"):
                sample_mode[0] = "vehicle"
            if k == ord("a"):
                sample_mode[0] = "aircraft"
            if k == ord("s"):
                fn = os.path.join(HERE, "snapshot_%d.png" % int(time.time()))
                cv2.imwrite(fn, out)
                print("[snapshot]", fn)
        if args.max_frames and frames >= args.max_frames:
            break

    link.send("CLEAR")
    logf.close()
    if writer:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()
    if leads:
        print("Warning lead times (s):", ", ".join("%.2f" % x for x in leads))


if __name__ == "__main__":
    main()
