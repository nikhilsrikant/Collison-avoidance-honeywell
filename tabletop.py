#!/usr/bin/env python3
"""Tabletop flight demo: tracks the model planes and serves the 3D escape display.

  python tabletop.py                       then open http://localhost:8765

What runs
  * Overhead camera + printed ArUco markers (markers_to_print.pdf) give each plane's position, nose
    heading, height above the table and speed. In a real aircraft, GPS + ADS-B provide this.
  * Arduino Uno + MPU-6500 in our plane (arduino/plane_lights_imu) gives roll and pitch, like a real
    attitude sensor (AHRS). The display sends back the escape intent, which drives the wingtip lights
    and, if the pilot is late, the auto-avoid servos.
  * Scale: 1 cm on the table = 25 m, 1 cm above the table = 50 ft, and time runs 4x faster, so moving a
    plane 10 cm per second looks like about 120 knots. All three are settings below.

Marker IDs: 1 = our plane (has the system), 2 = the private jet, 3 = another plane, 4 = runway (arrow = runway heading).
The LoRa / radio node ID of each plane must equal its marker ID.
Camera window keys: z = zero the table (both planes flat on the table), q = quit.
"""
import argparse
import math
import os
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

import server

KT = 0.514444
HERE = os.path.dirname(os.path.abspath(__file__))


def ang_diff(a, b):
    d = (a - b) % 360.0
    return d - 360.0 if d > 180 else d


# ----------------------------------------------------------------------------- camera tracking
class Tracker(threading.Thread):
    def __init__(self, args):
        super().__init__(daemon=True)
        self.a = args
        self.lock = threading.Lock()
        self.planes = {}          # marker id -> state
        self.table_z = None       # camera-to-table distance (cm), set by zeroing
        self.zero_samples = deque(maxlen=60)
        self.auto_zero_until = time.time() + 3.0 if args.auto_zero else 0
        self.preview = None
        self.fps = 0.0
        self.err = ""
        if args.camera_height_cm:
            self.table_z = float(args.camera_height_cm)

    def open(self):
        if self.a.video:
            cap = cv2.VideoCapture(self.a.video)
        else:
            backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
            cap = cv2.VideoCapture(self.a.camera, backend)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.a.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.a.height)
        return cap

    def zero(self):
        with self.lock:
            zs = [p["z_raw"] for p in self.planes.values() if time.time() - p["t"] < 0.5]
        if zs:
            self.table_z = float(np.median(zs))
            print("[zero] table is %.1f cm from the camera" % self.table_z)
            return True
        print("[zero] no marker in view; put the planes flat on the table under the camera")
        return False

    def run(self):
        cap = self.open()
        if not cap.isOpened():
            self.err = "camera not found (try --camera 1)"
            print("[camera]", self.err)
            return
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dic, params) if hasattr(cv2.aruco, "ArucoDetector") else None
        L = self.a.marker_cm
        obj = np.array([[-L / 2, L / 2, 0], [L / 2, L / 2, 0], [L / 2, -L / 2, 0], [-L / 2, -L / 2, 0]], dtype=np.float32)
        fps_src = cap.get(cv2.CAP_PROP_FPS) or 30.0
        next_t = time.time()
        n_frames, fps_t = 0, time.time()
        while True:
            ok, frame = cap.read()
            if not ok:
                if self.a.video:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    with self.lock:
                        self.planes.clear()
                    continue
                time.sleep(0.02)
                continue
            if self.a.video:  # play files in real time
                next_t += 1.0 / fps_src
                time.sleep(max(0.0, next_t - time.time()))
            t = time.time()
            H, W = frame.shape[:2]
            f = (W / 2.0) / math.tan(math.radians(self.a.hfov) / 2.0)
            K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1]], dtype=np.float32)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if detector is not None:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=params)
            out = frame.copy()
            if ids is not None:
                for c, mid in zip(corners, ids.flatten()):
                    mid = int(mid)
                    if mid not in self.a.ids:
                        continue
                    pts = c.reshape(4, 2).astype(np.float32)
                    okp, rvec, tvec = cv2.solvePnP(obj, pts, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not okp:
                        continue
                    X, Y, Z = [float(v) for v in tvec.reshape(3)]
                    top = (pts[0] + pts[1]) / 2.0
                    ctr = pts.mean(axis=0)
                    nose = math.degrees(math.atan2(top[0] - ctr[0], -(top[1] - ctr[1]))) % 360.0
                    self.update(mid, X, Y, Z, nose, t)
                    self.draw(out, mid, pts)
            if self.table_z is None and self.auto_zero_until and t < self.auto_zero_until:
                with self.lock:
                    for p in self.planes.values():
                        self.zero_samples.append(p["z_raw"])
            elif self.table_z is None and self.auto_zero_until and self.zero_samples:
                self.table_z = float(np.median(self.zero_samples))
                print("[zero] auto: table is %.1f cm from the camera (press z or click Zero table to redo)" % self.table_z)
            self.header(out)
            self.preview = out
            n_frames += 1
            if t - fps_t > 1.0:
                self.fps, n_frames, fps_t = n_frames / (t - fps_t), 0, t

    def update(self, mid, X, Y, Z, nose, t):
        a = self.a
        with self.lock:
            p = self.planes.get(mid)
            if p is None or t - p["t"] > 1.0:
                p = {"id": mid, "x": X, "y": Y, "z": Z, "z_raw": Z, "nose": nose, "t": t, "e": None, "n": None, "alt": 0.0,
                     "ve": 0.0, "vn": 0.0, "vs": 0.0, "psi": nose, "omega": 0.0, "first": t}
                self.planes[mid] = p
            # smooth the raw camera measurements (hand-held planes jitter)
            p["x"] += 0.5 * (X - p["x"])
            p["y"] += 0.5 * (Y - p["y"])
            p["z"] += 0.3 * (Z - p["z"])
            p["z_raw"] = Z
            p["nose"] = (p["nose"] + 0.5 * ang_diff(nose, p["nose"])) % 360.0
            alt_cm = max(0.0, self.table_z - p["z"]) if self.table_z else 0.0
            e = p["x"] * a.m_per_cm
            n = -p["y"] * a.m_per_cm
            alt_ft = alt_cm * a.ft_per_cm
            dt_sim = (t - p["t"]) * a.time_scale
            if p["e"] is not None and dt_sim > 1e-3:
                k = 0.3
                p["ve"] += k * ((e - p["e"]) / dt_sim - p["ve"])
                p["vn"] += k * ((n - p["n"]) / dt_sim - p["vn"])
                p["vs"] += k * ((alt_ft - p["alt"]) / dt_sim * 60.0 - p["vs"])
                gs = math.hypot(p["ve"], p["vn"])
                new_psi = math.degrees(math.atan2(p["ve"], p["vn"])) % 360.0 if gs > 15 else p["nose"]
                p["omega"] += 0.2 * (ang_diff(new_psi, p["psi"]) / dt_sim - p["omega"])
                p["psi"] = new_psi
            p["e"], p["n"], p["alt"], p["t"] = e, n, alt_ft, t

    def snapshot(self, imu):
        a = self.a
        now = time.time()
        out = []
        with self.lock:
            for mid, p in sorted(self.planes.items()):
                if p["e"] is None:
                    continue
                own = mid == a.own_id
                role = "own" if own else ("runway" if mid == a.runway_id else "other")
                rec = {
                    "id": mid, "role": role,
                    "label": "Our plane" if own else ("Runway" if role == "runway" else ("Private jet" if mid == 2 else "Plane %d" % mid)),
                    "e": round(p["e"], 1), "n": round(p["n"], 1), "alt_ft": round(p["alt"], 0),
                    "psi": round(p["psi"], 1), "nose": round(p["nose"], 1),
                    "gs_kt": round(math.hypot(p["ve"], p["vn"]) / KT, 1), "vs_fpm": round(p["vs"], 0),
                    "omega": round(max(-6.0, min(6.0, p["omega"])), 2), "age": round(now - p["t"], 2),
                    "roll": None, "pitch": None,
                }
                if own and imu.get("ok"):
                    rec["roll"], rec["pitch"] = imu["roll"], imu["pitch"]
                out.append(rec)
        return out

    def draw(self, img, mid, pts):
        own = mid == self.a.own_id
        col = (60, 220, 120) if own else (60, 170, 255)
        cv2.polylines(img, [pts.astype(np.int32)], True, col, 2)
        top = ((pts[0] + pts[1]) / 2).astype(int)
        c = pts.mean(axis=0).astype(int)
        cv2.arrowedLine(img, (int(c[0]), int(c[1])), (int(top[0]), int(top[1])), col, 2, tipLength=0.4)
        p = self.planes.get(mid)
        if p and p["e"] is not None:
            txt = "%s  %d ft  %d kt" % ("OUR PLANE" if own else "OTHER %d" % mid, p["alt"], math.hypot(p["ve"], p["vn"]) / KT)
            cv2.putText(img, txt, (int(c[0]) - 60, int(c[1]) - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(img, txt, (int(c[0]) - 60, int(c[1]) - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)

    def header(self, img):
        W = img.shape[1]
        cv2.rectangle(img, (0, 0), (W, 30), (18, 18, 18), -1)
        z = "table %.0f cm from camera" % self.table_z if self.table_z else "NOT ZEROED: planes flat on the table, press z"
        txt = "%.0f fps | %s | 1 cm = %g m, %g ft up | time x%g | z zero, q quit" % (
            self.fps, z, self.a.m_per_cm, self.a.ft_per_cm, self.a.time_scale)
        cv2.putText(img, txt, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)


from nodes import discover, NodeLink, RadioRelay  # noqa: E402  (shared with demo.py)


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="webcam index (1 for an external or phone camera)")
    ap.add_argument("--video", help="use a recorded video instead of a camera")
    ap.add_argument("--serial", default="auto", help="our plane's Arduino: auto, none, COM4, /dev/cu.usbmodem...")
    ap.add_argument("--jet-serial", default="auto", help="the jet's Arduino if it is on USB too: auto, none, COM5")
    ap.add_argument("--radio-loss", type=float, default=0.0, help="drop this share of relayed radio packets (0 to 1)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--marker-cm", type=float, default=8.0, help="printed marker size (black square edge)")
    ap.add_argument("--hfov", type=float, default=70.0, help="camera horizontal field of view in degrees")
    ap.add_argument("--camera-height-cm", type=float, default=0, help="skip zeroing if you measured it")
    ap.add_argument("--no-auto-zero", dest="auto_zero", action="store_false", help="do not zero during the first 3 s")
    ap.add_argument("--m-per-cm", type=float, default=25.0, help="meters of airspace per cm of table")
    ap.add_argument("--ft-per-cm", type=float, default=50.0, help="feet of altitude per cm above the table")
    ap.add_argument("--time-scale", type=float, default=4.0, help="how much faster than real time the airspace runs")
    ap.add_argument("--own-id", type=int, default=1)
    ap.add_argument("--runway-id", type=int, default=4, help="marker taped along the runway, arrow = runway heading")
    ap.add_argument("--ids", default="1,2,3,4", help="marker IDs to track")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--headless", action="store_true", help="no camera window")
    # alerting workflow
    ap.add_argument("--ta-s", type=float, default=40.0, help="traffic advisory at this tau (s)")
    ap.add_argument("--ra-s", type=float, default=25.0, help="resolution at this tau (s)")
    ap.add_argument("--pattern-ta", type=float, default=20.0, help="traffic advisory tau in the traffic pattern")
    ap.add_argument("--pattern-ra", type=float, default=15.0, help="resolution tau in the traffic pattern")
    ap.add_argument("--pattern-agl-ft", type=float, default=1000.0, help="below this height near the runway = pattern")
    ap.add_argument("--runway-hdg", type=float, default=None, help="runway heading if there is no runway marker")
    ap.add_argument("--hmd-ra-m", type=float, default=926.0, help="no resolution if the predicted miss is wider (m)")
    ap.add_argument("--min-descend-ft", type=float, default=300.0, help="never command a descent below this height")
    args = ap.parse_args()
    args.ids = {int(x) for x in args.ids.split(",")}

    import workflow
    tracker = Tracker(args)
    found = discover(args.serial, args.jet_serial, args.own_id)
    own = NodeLink("our plane", *found["own"]) if "own" in found else NodeLink("our plane")
    jet = NodeLink("jet", *found["jet"]) if "jet" in found else None
    relay = RadioRelay(args.radio_loss)
    if own.ser and own.hello.get("radio") == 0:
        relay.add(own)
    if jet and jet.hello.get("radio") == 0:
        relay.add(jet)
    if len(relay.links) == 1:
        print("[radio] one board: the other plane carries no electronics and is tracked by the camera only "
              "(like an aircraft with no system, seen on ADS-B or radar)")
    advisor = workflow.Advisor(args)
    adv_state = {"report": {"on": False}}

    def advisor_loop():
        while True:
            try:
                snap = tracker.snapshot(own.imu())
                rep = advisor.step(snap, own.node_state(relay.loss))
                rep["radio"]["relay"] = {"sent": relay.sent, "dropped": relay.dropped} if relay.links else None
                adv_state["report"] = rep
                own.write(advisor.node_command())
            except Exception as e:      # keep the loop alive during a demo
                print("[advisor]", repr(e))
            time.sleep(0.1)
    threading.Thread(target=advisor_loop, daemon=True).start()

    def get_tabletop(q):
        imu = own.imu()
        return {"time_scale": args.time_scale, "m_per_cm": args.m_per_cm, "ft_per_cm": args.ft_per_cm,
                "zeroed": tracker.table_z is not None, "fps": round(tracker.fps, 1), "error": tracker.err,
                "aircraft": tracker.snapshot(imu), "imu": imu,
                "jet_imu": jet.imu() if jet else None,
                "advisory": adv_state["report"], "raw": own.raw_state()}, 200

    def post_intent(d):
        own.set_intent(d)
        return {"ok": True, "sent": own.intent}, 200

    def post_zero(d):
        ok = tracker.zero()
        own.write("Z")
        if jet:
            jet.write("Z")
        return {"ok": ok, "table_cm": tracker.table_z}, 200

    def post_ack(d):
        own.write("ACK")
        return {"ok": True}, 200

    def post_ap(d):
        own.write("AP %d" % (1 if d.get("on") else 0))
        return {"ok": True}, 200

    def post_radio(d):
        try:
            relay.loss = max(0.0, min(1.0, float(d.get("loss", 0))))
        except (TypeError, ValueError):
            pass
        return {"ok": True, "loss": relay.loss}, 200

    server.EXTRA_GET["/api/tabletop"] = get_tabletop
    server.EXTRA_POST["/api/intent"] = post_intent
    server.EXTRA_POST["/api/zero"] = post_zero
    server.EXTRA_POST["/api/ack"] = post_ack
    server.EXTRA_POST["/api/ap"] = post_ap
    server.EXTRA_POST["/api/radio"] = post_radio
    server.PING_INFO["tabletop"] = True

    tracker.start()
    if args.headless:
        server.serve(args.port, None, "Tabletop flight demo", False)
        return
    threading.Thread(target=server.serve, args=(args.port, None, "Tabletop flight demo", False), daemon=True).start()
    win = "Tabletop camera"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    while True:
        if tracker.preview is not None:
            cv2.imshow(win, tracker.preview)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("z"):
            post_zero({})
        if not tracker.is_alive() and tracker.err:
            print("Camera stopped:", tracker.err)
            break
    own.write("N")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
