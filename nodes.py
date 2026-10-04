"""Serial links to the model planes' Arduinos (collision_node.ino or draft2.ino), shared by
demo.py (no camera) and tabletop.py (camera). Needs only pyserial."""
import threading
import time
import socket
import math
from collections import deque


# ----------------------------------------------------------------------------- the two Arduinos
SERIAL_HINTS = ("Arduino", "CH340", "CP210", "USB Serial", "USB-SERIAL")
PORT_HINTS = ("usbmodem", "usbserial", "wchusbserial", "ttyACM", "ttyUSB", "COM")


def _open(port):
    import serial
    if "://" in port:
        return serial.serial_for_url(port, baudrate=115200, timeout=0.05)
    s = serial.Serial(port, 115200, timeout=0.05)
    return s


class RangeFilter:
    """Steadies a jumpy range sensor (TF-Luna / HC-SR04) without hiding real motion.

    1. median of the last 5 readings     -> single spikes (a reflection, a glitch) vanish
    2. light exponential smoothing       -> the remaining noise shrinks
    3. 1.5 cm hold band                  -> a sensor resting still shows ONE steady number
    Readings of 0 or below count as "no echo"; 3 misses in a row clear the value.
    """

    def __init__(self, n=5, alpha=0.45, hold_cm=1.5):
        self.buf = deque(maxlen=n)
        self.alpha, self.hold = alpha, hold_cm
        self.smooth = None
        self.shown = None
        self.misses = 0

    def add(self, cm):
        if cm is None or cm <= 0:
            self.misses += 1
            if self.misses >= 3:
                self.buf.clear()
                self.smooth = self.shown = None
            return self.shown
        self.misses = 0
        self.buf.append(float(cm))
        med = sorted(self.buf)[len(self.buf) // 2]
        self.smooth = med if self.smooth is None else self.smooth + self.alpha * (med - self.smooth)
        if self.shown is None or abs(self.smooth - self.shown) >= self.hold:
            self.shown = self.smooth
        return self.shown


def discover(own_arg, jet_arg, own_id):
    """Create a Wi-Fi/UDP endpoint for the ESP32.

    The ESP32 broadcasts telemetry to UDP port 4210.  The first packet identifies
    the sender automatically, so no laptop IP or COM port is required here.
    """
    if own_arg == "none":
        return {}
    port = 4210
    try:
        if own_arg not in (None, "auto", "wifi", "udp"):
            port = int(own_arg)
    except ValueError:
        print("[nodes] --serial is now UDP; ignoring old serial value %r and using port 4210" % own_arg)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.10)
    except OSError as e:
        print("[nodes] could not listen on UDP %d: %s" % (port, e))
        return {}
    hello = {"id": own_id, "eq": 1, "radio": -1, "transport": "wifi"}
    print("[nodes] listening for ESP32 on Wi-Fi UDP port %d" % port)
    return {"own": (sock, "UDP:%d" % port, hello)}


class NodeLink(threading.Thread):
    """Serial link to one model plane's Arduino (collision_node.ino, or the old plane_lights_imu.ino)."""

    def __init__(self, name, ser=None, port=None, hello=None):
        super().__init__(daemon=True)
        self.name = name
        self.ser, self.port, self.hello = ser, port, hello or {}
        self.udp = isinstance(ser, socket.socket)
        self.peer_addr = None
        self._gyro_bias = None
        self._cal_samples = []
        self._last_imu_raw_t = None
        self.range_filter = RangeFilter()
        self.zero = None                # (roll, pitch, yaw) at "Zero IMU"; set automatically after calibration
        self.disp = None                # smoothed (roll, pitch) for the display
        self.roll = self.pitch = self.yaw = 0.0
        self.t = 0.0
        self.intent = "N"
        self.style = "B"
        self.last_sent = ""
        self.history = deque(maxlen=8)
        self.events = deque(maxlen=8)
        self.lock = threading.Lock()
        self.us = None                  # {"cm", "raw", "t"}
        self.us_hist = deque(maxlen=150)   # (time, cm or None), about 10 s
        self.counts = {"A": 0, "U": 0, "S": 0}
        self.rates = {"A": 0.0, "U": 0.0, "S": 0.0}
        self.rate_t = time.time()
        self.last_line_t = 0.0
        self.status = None              # parsed S line
        self.status_t = 0.0
        self.peers = {}                 # radio beacons heard: src -> {...}
        self.rx = 0
        self.on_radio_tx = None         # relay callback (RADIO_MODE 0)
        if self.ser:
            print("[%s] connected on %s" % (name, port))
            self.start()

    def write(self, line):
        if not self.ser:
            return
        with self.lock:
            try:
                if self.udp:
                    # the ESP32 sketch only understands "BUZZ <cm>" and reads one packet per 100 ms loop:
                    # anything else would only queue up in front of the buzzer commands
                    if self.peer_addr and line.startswith("BUZZ"):
                        self.ser.sendto((line + "\n").encode(), self.peer_addr)
                else:
                    self.ser.write((line + "\n").encode())
                if line != self.last_sent and not line.startswith(("T ", "X ")):
                    self.history.append("%s %s" % (time.strftime("%H:%M:%S"), line))
                if not line.startswith(("T ", "X ")):
                    self.last_sent = line
            except Exception as e:
                print("[%s] write failed: %s" % (self.name, e))

    def set_intent(self, d):
        style = "C" if d.get("style") == "C" else "B"
        if style != self.style:
            self.style = style
            self.write("S " + style)
        if d.get("mode") == "E":
            line = "E %s %s %d %d" % (d.get("turn", "S")[:1], d.get("vert", "H")[:1], int(d.get("hz", 3)), 1 if d.get("auto") else 0)
        else:
            line = "N"
        if line != self.intent:
            self.intent = line
            self.write(line)

    def _parse_raw_imu(self, s):
        """Accept ESP32 packets ax,ay,az,gx,gy,gz[,distance_cm] and estimate attitude.

        The optional seventh field is TF-Luna range in centimeters.
        """
        try:
            v = [float(x) for x in s.split(",")]
            if len(v) not in (6, 7):
                return False
            ax, ay, az, gx, gy, gz = v[:6]
            distance_cm = v[6] if len(v) == 7 else None
        except ValueError:
            return False
        now = time.time()
        self._tick_rates(now)
        if distance_cm is not None:
            cm = int(round(distance_cm))
            raw_cm = cm if cm > 0 else None
            f = self.range_filter.add(raw_cm)
            filt = round(f, 1) if f is not None else None
            self.us = {"cm": int(round(filt)) if filt is not None else None, "raw": raw_cm, "f": filt, "t": now}
            self.us_hist.append((now, filt, raw_cm))
            self.counts["U"] += 1
        # MPU6500 defaults after reset: accel +/-2g (16384 LSB/g), gyro +/-250 dps (131 LSB/dps).
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if self._gyro_bias is None:
            self._cal_samples.append((gx, gy, gz))
            if len(self._cal_samples) >= 20:
                n = len(self._cal_samples)
                self._gyro_bias = tuple(sum(x[i] for x in self._cal_samples) / n for i in range(3))
                self.roll, self.pitch = roll_acc, pitch_acc
                self.zero = (self.roll, self.pitch, self.yaw)     # whatever way the board sits now = level
                print("[%s] ESP32 IMU calibrated and zeroed (this position = level)" % self.name)
            self.t = now
            self.last_line_t = now
            return True
        dt = min(0.25, max(0.001, now - (self._last_imu_raw_t or now)))
        self._last_imu_raw_t = now
        bgx, bgy, bgz = self._gyro_bias
        gx = (gx - bgx) / 131.0
        gy = (gy - bgy) / 131.0
        gz = (gz - bgz) / 131.0
        self.roll = 0.98 * (self.roll + gx * dt) + 0.02 * roll_acc
        self.pitch = 0.98 * (self.pitch + gy * dt) + 0.02 * pitch_acc
        self.yaw = (self.yaw + gz * dt) % 360.0
        self._update_disp()
        self.t = now
        self.last_line_t = now
        self.counts["A"] += 1
        return True

    def _tick_rates(self, now):
        if now - self.rate_t >= 1.0:
            for k in self.counts:
                self.rates[k] = round(self.counts[k] / (now - self.rate_t), 1)
                self.counts[k] = 0
            self.rate_t = now

    def zero_now(self):
        """Zero IMU button: the board's current attitude becomes level (the ESP32 itself has no zero)."""
        self.zero = (self.roll, self.pitch, self.yaw)
        self.disp = None
        self.events.append("%s IMU zeroed" % time.strftime("%H:%M:%S"))

    def _update_disp(self):
        """Called once per new IMU sample: zero it and smooth it lightly (about 0.25 s at 10 Hz)."""
        z = self.zero or (0.0, 0.0, 0.0)
        r, p = -(self.roll - z[0]), self.pitch - z[1]
        if self.disp is None:
            self.disp = [r, p]
        else:
            self.disp[0] += 0.4 * (r - self.disp[0])
            self.disp[1] += 0.4 * (p - self.disp[1])

    def attitude(self):
        """Zeroed, smoothed (roll, pitch, yaw) in degrees, signs as the dashboard expects."""
        if self.disp is None:
            self._update_disp()
        z = self.zero or (0.0, 0.0, 0.0)
        return self.disp[0], self.disp[1], (self.yaw - z[2]) % 360.0

    def parse(self, s):
        now = time.time()
        if self._parse_raw_imu(s):
            return
        if s:
            self.last_line_t = now
        self._tick_rates(now)
        if s.startswith("A "):
            try:
                self.roll, self.pitch, self.yaw = [float(v) for v in s[2:].split()]
                self._update_disp()
                self.t = now
                self.counts["A"] += 1
            except ValueError:
                pass
        elif s.startswith("U "):
            try:
                p = s.split()
                cm = int(p[1])
                raw = int(p[2]) if len(p) > 2 else cm
                self.us = {"cm": cm if cm > 0 else None, "raw": raw if raw > 0 else None, "t": now}
                self.us_hist.append((now, cm if cm > 0 else None, raw if raw > 0 else None))
                self.counts["U"] += 1
            except (ValueError, IndexError):
                pass
        elif s.startswith("S "):
            p = s.split()
            if len(p) >= 12:
                try:
                    self.status = {"stage": int(p[1]), "vert": p[2], "turn": p[3], "neg": p[4], "ack": int(p[5]),
                                   "ap": int(p[6]), "auto": int(p[7]), "comply": int(p[8]), "peer": int(p[9]),
                                   "peer_eq": int(p[10]), "ra_by": p[11],
                                   "buzz": p[12] if len(p) > 12 else "?"}
                    self.status_t = now
                    self.counts["S"] += 1
                except ValueError:
                    pass
        elif s.startswith("R "):
            p = s.split()
            try:
                src = int(p[1])
                self.peers[src] = {"type": int(p[2]), "eq": int(p[3]), "pitch": int(p[4]), "roll": int(p[5]),
                                   "flags": int(p[6]), "rssi": int(p[7]), "t": now}
                self.rx += 1
            except (IndexError, ValueError):
                pass
        elif s.startswith("X "):
            if self.on_radio_tx:
                self.on_radio_tx(self, s[2:].strip())
        elif s.startswith("HELLO "):
            p = s.split()
            try:
                self.hello = {"id": int(p[1]), "eq": int(p[2]), "radio": int(p[3])}
            except (IndexError, ValueError):
                pass
        elif s.startswith("EV "):
            msg = s[3:]
            self.events.append("%s %s" % (time.strftime("%H:%M:%S"), msg))
            print("[%s] %s" % (self.name, msg))
        elif s and not s.startswith(("E ", "N", "S ")):
            print("[%s] %s" % (self.name, s))

    def run(self):
        buf = b""
        last_beat = 0.0
        while self.ser:
            try:
                if self.udp:
                    data, addr = self.ser.recvfrom(2048)
                    self.peer_addr = addr
                    # UDP packets are normally one telemetry record each; also accept newline batches.
                    for line in data.decode(errors="ignore").replace("\r", "").split("\n"):
                        if line.strip():
                            self.parse(line.strip())
                else:
                    buf += self.ser.read(256)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.parse(line.decode(errors="ignore").strip())
            except socket.timeout:
                pass
            except Exception as e:
                print("[%s] receive error: %s" % (self.name, e))
                time.sleep(0.1)
            if time.time() - last_beat > 1.0:
                last_beat = time.time()
                self.write(self.intent)

    def imu(self):
        ok = self.ser is not None and time.time() - self.t < 1.0
        r, p, y = self.attitude()
        return {"ok": ok, "roll": round(r, 1), "pitch": round(p, 1), "yaw": round(y, 1), "port": self.port,
                "last_cmd": self.last_sent, "cmd_log": list(self.history)}

    def raw_state(self):
        """Everything the board reports, unprocessed, for the dashboard's raw panel."""
        now = time.time()
        st = self.status if self.status and now - self.status_t < 0.6 else None
        us = self.us if self.us and now - self.us["t"] < 1.0 else None
        return {
            "connected": self.ser is not None, "port": self.port, "hello": self.hello,
            "alive": self.ser is not None and now - self.last_line_t < 1.0,
            "rates": self.rates,
            "us_cm": us["cm"] if us else None, "us_raw": us["raw"] if us else None, "us_fresh": us is not None,
            "us_hist": [[round(h[0] - now, 2), h[1], h[2] if len(h) > 2 else h[1]] for h in self.us_hist if now - h[0] < 10.0],
            "roll": self.imu()["roll"], "pitch": self.imu()["pitch"], "yaw": self.imu()["yaw"],
            "zeroed": self.zero is not None,
            "imu_ok": now - self.t < 1.0,
            "stage": st["stage"] if st else None, "buzz": st["buzz"] if st else None,
            "ack": st["ack"] if st else None, "comply": st["comply"] if st else None,
            "events": list(self.events),
        }

    def node_state(self, loss=0.0):
        """What the advisor needs from our plane."""
        now = time.time()
        st = self.status if self.status and now - self.status_t < 0.6 else None
        us = None
        if self.us:
            us = {"cm": self.us.get("f", self.us["cm"]), "age": now - self.us["t"]}
        peers = {k: v for k, v in self.peers.items() if now - v["t"] < 3.0}
        return {"status": st, "us": us, "peers": peers, "rx": self.rx, "events": list(self.events),
                "radio": {0: "USB relay", 1: "LoRa"}.get(self.hello.get("radio"), "none") if self.ser else "none",
                "loss": loss}


class RadioRelay:
    """RADIO_MODE 0: no LoRa modules yet, so the laptop carries the packets between the two USB ports.
    --radio-loss drops a share of them, to show the tie-break still gives opposite senses."""

    def __init__(self, loss=0.0):
        self.loss = loss
        self.links = []
        self.sent = self.dropped = 0

    def add(self, link):
        self.links.append(link)
        link.on_radio_tx = self.tx

    def tx(self, src, hexpkt):
        import random
        for dst in self.links:
            if dst is src:
                continue
            self.sent += 1
            if random.random() < self.loss:
                self.dropped += 1
                continue
            dst.write("X " + hexpkt)
