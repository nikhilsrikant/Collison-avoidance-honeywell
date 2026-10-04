"""3D airliner view of a model plane's attitude, plus the collision workflow state.

Same airliner model as the first version. What changed:
  * The Arduino (collision_node.ino) now runs the complementary filter and sends roll, pitch and yaw,
    so this script no longer calibrates or filters.
  * The USB port belongs to tabletop.py now, so by default this script reads everything from
    tabletop.py's web API. The warning text is the real alert (traffic / resolution / escalation),
    not just the ultrasonic distance.

  python mpu6500_3d.py                 uses tabletop.py if running, otherwise listens for ESP32 UDP
  python mpu6500_3d.py --plane jet     the private jet's attitude (through tabletop.py)
  python mpu6500_3d.py --udp-port 4210 listen for raw ESP32 MPU6500 packets
"""
import argparse
import json
import math
import socket
import time
import urllib.request

from vpython import (
    canvas,
    vector,
    cylinder,
    cone,
    ellipsoid,
    triangle,
    vertex,
    ring,
    compound,
    color,
    label,
    rate,
    cross,
    norm,
    mag
)

ap = argparse.ArgumentParser()
ap.add_argument("--api", default="http://localhost:8765", help="where tabletop.py is running")
ap.add_argument("--plane", choices=["own", "jet"], default="own", help="which plane to show")
ap.add_argument("--serial", default=None, help="legacy USB serial mode")
ap.add_argument("--udp-port", type=int, default=4210, help="UDP port for ESP32 IMU data")
ap.add_argument("--warn-cm", type=float, default=30.0, help="--serial mode: warning distance")
args = ap.parse_args()

print("--------------------------------")
print("Aircraft attitude + collision workflow")
print("--------------------------------")

# ============================================================
# VPYTHON SCENE
# ============================================================

scene = canvas(
    title="Aircraft Collision Demo",
    width=900,
    height=600,
    background=vector(
        0.08,
        0.08,
        0.08
    )
)

scene.range = 6

scene.forward = vector(
    -1,
    -0.3,
    -1
)


# ============================================================
# AIRCRAFT MODEL
# ============================================================

# ============================================================
# SMALL COMMERCIAL AIRLINER MODEL
# ============================================================

# Main fuselage
fuselage = cylinder(
    pos=vector(-3.2, 0, 0),
    axis=vector(6.0, 0, 0),
    radius=0.48,
    color=vector(0.82, 0.84, 0.86)
)

# Rounded / pointed nose
nose = cone(
    pos=vector(2.8, 0, 0),
    axis=vector(1.0, 0, 0),
    radius=0.47,
    color=vector(0.82, 0.84, 0.86)
)

# ============================================================
# COCKPIT WINDOWS
# ============================================================

cockpit = ellipsoid(
    pos=vector(2.65, 0.28, 0),
    length=0.65,
    height=0.25,
    width=0.65,
    color=vector(0.08, 0.15, 0.22)
)


# ============================================================
# MAIN WINGS
# ============================================================

left_wing = triangle(
    v0=vertex(
        pos=vector(0.7, 0, 0.25),
        color=vector(0.70, 0.72, 0.75)
    ),

    v1=vertex(
        pos=vector(-1.3, 0, 3.8),
        color=vector(0.70, 0.72, 0.75)
    ),

    v2=vertex(
        pos=vector(-1.5, 0, 0.25),
        color=vector(0.70, 0.72, 0.75)
    )
)


right_wing = triangle(
    v0=vertex(
        pos=vector(0.7, 0, -0.25),
        color=vector(0.70, 0.72, 0.75)
    ),

    v1=vertex(
        pos=vector(-1.3, 0, -3.8),
        color=vector(0.70, 0.72, 0.75)
    ),

    v2=vertex(
        pos=vector(-1.5, 0, -0.25),
        color=vector(0.70, 0.72, 0.75)
    )
)


# ============================================================
# HORIZONTAL TAIL
# ============================================================

left_tail = triangle(
    v0=vertex(
        pos=vector(-2.5, 0.15, 0.15),
        color=vector(0.68, 0.70, 0.73)
    ),

    v1=vertex(
        pos=vector(-3.2, 0.15, 1.6),
        color=vector(0.68, 0.70, 0.73)
    ),

    v2=vertex(
        pos=vector(-3.3, 0.15, 0.15),
        color=vector(0.68, 0.70, 0.73)
    )
)


right_tail = triangle(
    v0=vertex(
        pos=vector(-2.5, 0.15, -0.15),
        color=vector(0.68, 0.70, 0.73)
    ),

    v1=vertex(
        pos=vector(-3.2, 0.15, -1.6),
        color=vector(0.68, 0.70, 0.73)
    ),

    v2=vertex(
        pos=vector(-3.3, 0.15, -0.15),
        color=vector(0.68, 0.70, 0.73)
    )
)


# ============================================================
# VERTICAL TAIL
# ============================================================

vertical_tail = triangle(
    v0=vertex(
        pos=vector(-2.5, 0.2, 0),
        color=vector(0.2, 0.4, 0.75)
    ),

    v1=vertex(
        pos=vector(-3.15, 1.8, 0),
        color=vector(0.2, 0.4, 0.75)
    ),

    v2=vertex(
        pos=vector(-3.35, 0.2, 0),
        color=vector(0.2, 0.4, 0.75)
    )
)


# ============================================================
# LEFT ENGINE
# ============================================================

left_engine = cylinder(
    pos=vector(-0.4, -0.45, 1.35),
    axis=vector(1.25, 0, 0),
    radius=0.35,
    color=vector(0.35, 0.37, 0.40)
)

left_engine_front = ring(
    pos=vector(0.85, -0.45, 1.35),
    axis=vector(1, 0, 0),
    radius=0.30,
    thickness=0.08,
    color=vector(0.15, 0.15, 0.15)
)


# ============================================================
# RIGHT ENGINE
# ============================================================

right_engine = cylinder(
    pos=vector(-0.4, -0.45, -1.35),
    axis=vector(1.25, 0, 0),
    radius=0.35,
    color=vector(0.35, 0.37, 0.40)
)

right_engine_front = ring(
    pos=vector(0.85, -0.45, -1.35),
    axis=vector(1, 0, 0),
    radius=0.30,
    thickness=0.08,
    color=vector(0.15, 0.15, 0.15)
)


# ============================================================
# COMBINE AIRCRAFT
# ============================================================

aircraft = compound([
    fuselage,
    nose,
    cockpit,

    left_wing,
    right_wing,

    left_tail,
    right_tail,

    vertical_tail,

    left_engine,
    left_engine_front,

    right_engine,
    right_engine_front
])

# ============================================================
# TEXT DISPLAY
# ============================================================

distance_label = label(
    pos=vector(0, 4.0, 0),
    text="Distance: waiting...",
    height=20,
    box=False,
    color=color.white
)


warning_label = label(
    pos=vector(0, 3.3, 0),
    text="",
    height=26,
    box=False,
    color=color.red
)


status_label = label(
    pos=vector(0, -4.0, 0),
    text="Waiting for sensor data...",
    height=14,
    box=False,
    color=color.gray(0.7)
)



# ============================================================
# DATA SOURCES
# ============================================================

class ApiSource:
    """Reads tabletop.py: attitude from the IMU, alert state from the workflow."""

    def __init__(self, base, plane):
        self.url = base.rstrip("/") + "/api/tabletop"
        self.plane = plane
        self.last_ok = 0.0
        print("Reading", self.url, "(" + ("private jet" if plane == "jet" else "our plane") + ")")

    def read(self):
        try:
            with urllib.request.urlopen(self.url, timeout=0.5) as r:
                d = json.load(r)
        except Exception as e:
            return {"error": "tabletop.py not answering (%s)" % e.__class__.__name__}
        imu = d.get("jet_imu") if self.plane == "jet" else d.get("imu")
        adv = d.get("advisory") or {}
        if not imu or not imu.get("ok"):
            return {"error": "no IMU data from the %s's Arduino" % ("jet" if self.plane == "jet" else "plane")}
        out = {"roll": imu["roll"], "pitch": imu["pitch"], "yaw": imu.get("yaw", 0.0),
               "distance": adv.get("us_cm") if self.plane == "own" else None}
        if adv.get("on") and self.plane == "own":
            b = adv.get("banner", {})
            out["level"] = adv.get("level", 0)
            out["stage"] = adv.get("stage", 0)
            out["state"] = b.get("state", "")
            out["msg"] = b.get("msg", "").replace("<em>", "").replace("</em>", "")
            out["tau"] = adv.get("tau")
        return out


class UdpSource:
    """Receive raw MPU6500 CSV packets from the ESP32 over Wi-Fi UDP.

    Expected packet: ax,ay,az,gx,gy,gz
    The ESP32 sketch uses the MPU6500 default ranges: accel +/-2 g and gyro +/-250 deg/s.
    """
    def __init__(self, port=4210):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "distance": None}
        self.last_time = None
        self.last_packet = 0.0
        self.cal_start = time.time()
        self.bias_samples = []
        self.gx_bias = self.gy_bias = self.gz_bias = 0.0
        self.calibrated = False
        self.alpha = 0.98
        print("Listening for ESP32 IMU data on UDP port %d..." % port)
        print("Keep the MPU6500 still for about 2 seconds when data starts.")

    def _process(self, text):
        parts = text.strip().split(",")
        if len(parts) != 6:
            return
        try:
            axr, ayr, azr, gxr, gyr, gzr = map(float, parts)
        except ValueError:
            return

        # ESP32 sketch leaves MPU6500 at its default ranges.
        ax, ay, az = axr / 16384.0, ayr / 16384.0, azr / 16384.0
        gx, gy, gz = gxr / 131.0, gyr / 131.0, gzr / 131.0
        now = time.time()
        self.last_packet = now

        # Estimate gyro zero bias during the first ~2 seconds of received data.
        if not self.calibrated:
            self.bias_samples.append((gx, gy, gz))
            if len(self.bias_samples) >= 20 and now - self.cal_start >= 2.0:
                n = len(self.bias_samples)
                self.gx_bias = sum(v[0] for v in self.bias_samples) / n
                self.gy_bias = sum(v[1] for v in self.bias_samples) / n
                self.gz_bias = sum(v[2] for v in self.bias_samples) / n
                self.calibrated = True
                self.last_time = now
                self.state["roll"] = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
                self.state["pitch"] = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
                print("IMU calibrated. Aircraft visualization active.")
            return

        gx -= self.gx_bias
        gy -= self.gy_bias
        gz -= self.gz_bias
        dt = max(0.001, min(now - self.last_time, 0.25)) if self.last_time else 0.1
        self.last_time = now

        acc_roll = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
        acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
        self.state["roll"] = self.alpha * (self.state["roll"] + gx * dt) + (1-self.alpha) * acc_roll
        self.state["pitch"] = self.alpha * (self.state["pitch"] + gy * dt) + (1-self.alpha) * acc_pitch
        self.state["yaw"] += gz * dt

    def read(self):
        got = False
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                got = True
                self._process(data.decode(errors="ignore"))
            except BlockingIOError:
                break
            except OSError as e:
                return {"error": "UDP error: %s" % e}
        if not self.calibrated:
            if not got and self.last_packet == 0:
                return {"error": "Waiting for ESP32 on UDP port %d..." % self.port}
            return {"error": "Calibrating IMU - keep it still..."}
        if time.time() - self.last_packet > 1.5:
            return {"error": "ESP32 data stopped - waiting for Wi-Fi packets..."}
        return dict(self.state)

    def close(self):
        self.sock.close()


def pick_source():
    """Use tabletop.py when available; otherwise listen to the ESP32 over Wi-Fi UDP."""
    try:
        urllib.request.urlopen(args.api.rstrip("/") + "/api/ping", timeout=0.25).read()
        print("tabletop.py found: using its API.")
        return ApiSource(args.api, args.plane)
    except Exception:
        return UdpSource(args.udp_port)


source = pick_source()


# ============================================================
# ORIENTATION (same math as before; angles now come filtered from the Arduino)
# ============================================================

def orient(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    forward = vector(math.cos(pitch) * math.cos(yaw), math.sin(pitch), math.cos(pitch) * math.sin(yaw))
    right = cross(forward, vector(0, 1, 0))
    right = vector(0, 0, 1) if mag(right) < 0.001 else norm(right)
    up_without_roll = norm(cross(right, forward))
    up = up_without_roll * math.cos(roll) + right * math.sin(roll)
    aircraft.axis = forward
    aircraft.up = up


LEVEL_COLOR = {0: color.white, 1: vector(1.0, 0.71, 0.15), 2: vector(1.0, 0.3, 0.3)}


# ============================================================
# MAIN LOOP
# ============================================================

print("3D view running. Ctrl+C to stop.")
last_poll = 0.0
warn_on = False
try:
    while True:
        rate(50)
        now = time.time()
        if isinstance(source, ApiSource) and now - last_poll < 0.05:   # 20 polls a second is plenty
            continue
        if isinstance(source, UdpSource) and now - last_poll < 0.02:
            continue
        last_poll = now
        d = source.read()

        if "error" in d:
            status_label.text = d["error"]
            continue

        orient(d["roll"], d["pitch"], d["yaw"])

        dist = d.get("distance")
        distance_label.text = "Distance: No echo" if dist is None else "Distance: %.0f cm" % dist

        if "level" in d:                                   # full workflow from tabletop.py
            stage = d.get("stage", 0)
            if d["level"] == 0 and stage < 2:
                warning_label.text = ""
            else:
                tau = d.get("tau")
                warning_label.text = d["state"].upper() + ": " + d["msg"] + ("  (tau %.0f s)" % tau if tau is not None else "")
            warning_label.color = vector(0.9, 0.36, 0.94) if "Autopilot" in d.get("state", "") else LEVEL_COLOR.get(max(d["level"], 2 if stage >= 2 else 0), color.red)
        else:                                              # --serial / jet view: proximity only, like before
            warning_label.color = color.red
            if dist is not None and dist < args.warn_cm:
                warn_on = True
            elif dist is None or dist > args.warn_cm + 3:
                warn_on = False
            warning_label.text = "COLLISION WARNING" if warn_on else ""

        status_label.text = "Roll: %.1f deg    Pitch: %.1f deg    Yaw: %.1f deg" % (d["roll"], d["pitch"], d["yaw"])

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    if isinstance(source, UdpSource):
        try:
            source.close()
        except Exception:
            pass
    print("Program exited cleanly.")
