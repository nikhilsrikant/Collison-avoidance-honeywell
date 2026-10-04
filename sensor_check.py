#!/usr/bin/env python3
"""Raw check of the board, no dashboard: what the Arduino actually sends, and what is wrong.

  python sensor_check.py              finds the Arduino by itself, runs 20 s
  python sensor_check.py COM7 60      a given port, 60 s

Close demo.py, tabletop.py, mpu6500_3d.py and the Serial Monitor first: only one program can use the port.
Works with collision_node.ino, draft2.ino, and tells you if draft1.ino is still on the board.
While it runs: move a hand toward the HC-SR04 and away, tilt the board. Ctrl+C stops early.
"""
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing: python -m pip install pyserial")

HINTS = ("Arduino", "CH340", "CP210", "USB Serial", "USB-SERIAL", "usbmodem", "usbserial", "ttyACM", "ttyUSB")
BUZZ = {"-": "off", "3": "ON (resolution not followed)", "4": "ON (ACK required)", "A": "tick (autopilot)",
        "P": "ON (closer than 30 cm, no laptop)"}


def find_port():
    for p in list_ports.comports():
        if any(h in (p.description or "") + (p.manufacturer or "") + p.device for h in HINTS):
            return p.device
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    if not port:
        sys.exit("No Arduino found. Plug it in, or run: python sensor_check.py COM7")
    print("Opening %s at 115200 (the Uno restarts; keep it still 2 s)..." % port)
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except serial.SerialException as e:
        sys.exit("Cannot open %s: %s\nClose demo.py / tabletop.py / the Serial Monitor and try again." % (port, e))

    hello, events = None, []
    n = {"A": 0, "U": 0, "S": 0, "csv": 0, "junk": 0}
    us_vals, us_none, last = [], 0, {}
    pitch_vals = []
    buzz_seen = {}
    buf, t0, t_print = b"", time.time(), 0.0
    t_first = None
    try:
        while time.time() - t0 < secs:
            buf += ser.read(512)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode(errors="ignore").strip()
                p = s.split()
                if not s:
                    continue
                if t_first is None and (s.startswith(("A ", "U ")) or s.count(",") == 6):
                    t_first = time.time()
                if s.startswith("HELLO"):
                    hello = s
                elif s.startswith("EV "):
                    events.append(s[3:])
                    print("   board says:", s[3:])
                elif s.startswith("A ") and len(p) == 4:
                    n["A"] += 1
                    last["imu"] = (float(p[1]), float(p[2]), float(p[3]))
                    pitch_vals.append(float(p[2]))
                elif s.startswith("U "):
                    n["U"] += 1
                    cm = int(p[1])
                    raw = int(p[2]) if len(p) > 2 else cm
                    last["us"] = (cm, raw)
                    if cm > 0:
                        us_vals.append(cm)
                    else:
                        us_none += 1
                elif s.startswith("S ") and len(p) >= 12:
                    n["S"] += 1
                    b = p[12] if len(p) > 12 else "?"
                    buzz_seen[b] = buzz_seen.get(b, 0) + 1
                    last["st"] = (int(p[1]), b)
                elif s.count(",") == 6:
                    n["csv"] += 1
                else:
                    n["junk"] += 1
            now = time.time()
            if now - t_print > 0.25:
                t_print = now
                imu = last.get("imu")
                us = last.get("us")
                st = last.get("st")
                print("\r%5.1fs | distance %-14s | roll %6s pitch %6s | stage %s | buzzer %-30s" % (
                    now - t0,
                    ("%d cm (raw %s)" % (us[0], us[1] if us[1] > 0 else "-")) if us and us[0] > 0 else ("no echo" if us else "NO DATA"),
                    "%.1f" % imu[0] if imu else "-", "%.1f" % imu[1] if imu else "-",
                    st[0] if st else "-", BUZZ.get(st[1], st[1]) if st else "-"), end="", flush=True)
    except KeyboardInterrupt:
        pass
    ser.close()
    dur = max(1.0, time.time() - (t_first or t0))
    print("\n\n=== Summary ===")
    print("HELLO:", hello or "none")
    print("IMU lines %.0f/s, distance lines %.0f/s, status lines %.0f/s" % (n["A"] / dur, n["U"] / dur, n["S"] / dur))

    problems = []
    if n["csv"] and not (n["A"] or n["U"]):
        problems.append("The board still runs draft1.ino (CSV). Upload collision_node.ino (or draft2.ino).")
    if not hello and not n["csv"]:
        problems.append("No HELLO line: wrong sketch, or wrong baud. The new sketches talk at 115200.")
    if n["A"] == 0:
        problems.append("No IMU data: check MPU SDA->A4, SCL->A5, VCC, GND.")
    elif pitch_vals and max(pitch_vals) - min(pitch_vals) < 1.0:
        problems.append("Pitch never changed: did you tilt the board? If you did, the MPU may not be updating.")
    if n["U"] == 0:
        if hello and hello.split()[1:3] == ["2", "0"]:
            problems.append("This board is set up as the jet (EQUIPPED 0): it has no ultrasonic. Use NODE_ID 1, EQUIPPED 1.")
        else:
            problems.append("No distance lines at all: the sketch is not reading the HC-SR04 (HAS_ULTRASONIC 0?).")
    elif not us_vals:
        problems.append("HC-SR04 never got an echo: check TRIG->D9, ECHO->D10, VCC->5V, GND. "
                        "Point it at a wall 30-150 cm away, not along the table.")
    else:
        lo, hi = min(us_vals), max(us_vals)
        print("Distance seen: %d to %d cm (%d echoes, %d without echo)" % (lo, hi, len(us_vals), us_none))
        if hi - lo < 3:
            problems.append("Distance stuck at about %d cm: the sensor sees something fixed (the table, a wire, "
                            "the breadboard). Lift it off the table and aim it straight ahead." % lo)
        if lo < 30 and "P" in buzz_seen:
            print("Note: buzzer was on because something was closer than 30 cm with no laptop program running.")
    if buzz_seen:
        print("Buzzer reasons seen:", ", ".join("%s x%d" % (BUZZ.get(k, k), v) for k, v in buzz_seen.items()))
        if "?" in buzz_seen:
            problems.append("Old collision_node.ino without the buzzer reason: upload the new one.")
    if problems:
        print("\nProblems found:")
        for p in problems:
            print(" -", p)
    else:
        print("\nAll good: IMU, distance and status are coming through.")


if __name__ == "__main__":
    main()
