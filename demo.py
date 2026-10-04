#!/usr/bin/env python3
"""Collision avoidance demo with NO camera: one ESP32 over Wi-Fi UDP, IMU + HC-SR04.

  python demo.py                 then open http://localhost:8765 in Chrome
  python demo.py                 ESP32 telemetry is received on UDP port 4210

Setup
  * Smart plane (Airbus / Boeing): the Arduino with collision_node.ino (or draft2.ino). The HC-SR04
    points forward out of the nose. The IMU is fixed flat to the plane.
  * Non-smart plane: nothing on it. It is detected only by our nose sensor, the way an aircraft with
    no system is detected by radar.

How it works
  * Range = the HC-SR04 distance. An alpha-beta filter gives the closing rate, tau = range / closing rate
    (TCAS modified tau). The same workflow as the camera version then runs: traffic advisory at 40 s,
    resolution at 25 s, escalation, ACK, autopilot gate, clear of conflict.
  * Scale (same as before): 1 cm = 25 m, time runs 4x, so 10 cm per second of closing speed looks like
    about 120 knots. The traffic advisory comes at roughly 1.1 m, the resolution at roughly 75 cm.

Limits without a camera (say these in Q&A)
  * The sensor only sees straight ahead (about 15 degrees), so the demo is a head-on approach.
  * It cannot see the other plane's altitude, so the resolution is CLIMB by default (--sense descend
    to change it), and the turn is RIGHT (head-on rule, 14 CFR 91.113).
  * Our own climb or descent on the display comes from the IMU pitch.
"""
import argparse
import math
import threading
import time
from collections import deque

import server
import workflow
from nodes import discover, NodeLink

KT = 0.514444


class RangeTracker:
    """Range and closing rate from the (already filtered) distance sensor.

    The closing rate is the slope of a straight line fitted to the last 2 s of readings. It only counts
    as motion when the slope is clearly bigger than the sensor's own scatter (3.5 standard errors), at
    least --min-closing-cm-s, the line moved 10 cm or more, and that held for about half a second. A sensor resting on the table reads
    "steady" (0 cm/s) instead of jumping around and setting off false alerts.
    """

    WINDOW = 2.0                   # seconds of readings in each fit
    K = 3.5                        # slope must be this many standard errors from zero
    MOVED = 10.0                   # and the fitted line must move at least this many cm
    PERSIST = 5                    # and point the same way in this many fits in a row (about 0.5 s)
    GAP = 1.5                      # s without a reading before the fit starts over
    VALID = 1.5                    # s the last reading still counts (weak returns come in short gaps)

    def __init__(self, a):
        self.a = a
        self.samples = deque()     # (t, cm)
        self.r = None              # cm
        self.rdot = 0.0            # cm/s, negative = closing
        self.seen_t = 0.0
        self.last_t = None
        self.streak = 0
        self.last_sign = 0

    def update(self, us_entry):
        if not us_entry:
            return
        t = us_entry["t"]
        if t == self.last_t:
            return                 # nothing new since the last call
        self.last_t = t
        cm = us_entry.get("f")
        if cm is None:
            return
        # in range up to --max-cm, with a 15 cm band so an object sitting right at the edge does not flicker
        if cm > self.a.max_cm + (15.0 if self.valid() else 0.0):
            return
        if self.samples and t - self.samples[-1][0] > self.GAP:
            self.samples.clear()   # long gap: start a fresh fit
            self.streak, self.last_sign, self.rdot = 0, 0, 0.0
        self.samples.append((t, cm))
        while self.samples and t - self.samples[0][0] > self.WINDOW:
            self.samples.popleft()
        self.r, self.seen_t = cm, t
        n = len(self.samples)
        span = self.samples[-1][0] - self.samples[0][0] if n else 0.0
        if n >= 6 and span >= 0.7:
            mt = sum(x[0] for x in self.samples) / n
            mc = sum(x[1] for x in self.samples) / n
            den = sum((x[0] - mt) ** 2 for x in self.samples)
            slope = sum((x[0] - mt) * (x[1] - mc) for x in self.samples) / den if den > 0 else 0.0
            # how sure are we of that slope? standard error from the scatter around the fitted line
            resid = sum((x[1] - (mc + slope * (x[0] - mt))) ** 2 for x in self.samples)
            se = (resid / max(1, n - 2) / den) ** 0.5 if den > 0 else 99.0
            moved = abs(slope) * span                      # cm the fitted line moved across the window
            real = abs(slope) >= max(self.a.min_closing_cm_s, self.K * se) and moved >= self.MOVED
            # noise makes short bursts; real motion lasts: require the same direction in PERSIST fits in a row
            sign = (1 if slope > 0 else -1) if real else 0
            self.streak = self.streak + 1 if (sign != 0 and sign == self.last_sign) else (1 if sign else 0)
            self.last_sign = sign
            if real and self.streak >= self.PERSIST:
                self.rdot = slope
            elif not real:
                self.rdot = 0.0
            # (real but not yet PERSIST fits in a row after a short gap: keep the last rate, do not drop to 0)
        # too few points right after a gap: keep the last rate instead of pretending it stopped

    def valid(self):
        return self.r is not None and time.time() - self.seen_t < self.VALID


class VirtualNode:
    """The escalation and autopilot logic, run on the laptop because the ESP32 only sends sensor data.
    Same rules and timings as collision_node.ino had on the board:
      resolution issued -> 3 s to start following (IMU pitch past +-5 deg) -> buzzer
      -> 5 s -> "Acknowledge" -> automatic avoid (if allowed) when still not following, or earlier at the
      dashboard's calculated last safe moment (minus 1 s real), with tau <= 16 s as a hard floor
    Auto-avoid policy comes from the dashboard's AUTO-AVOID menu: off / backup / always.
    The pilot takes control back by tilting against the escape (15 deg), pressing X, or the button."""

    GRACE, SHAKER, AUTO_TAU, PITCH_OK, ROLL_OK, OVERRIDE = 3.0, 5.0, 16.0, 5.0, 10.0, 15.0
    LSM_MARGIN = 4.0              # airspace seconds (1 s real at 4x) before the calculated last safe moment:
                                  # covers the sensor filter, the 0.25 s plan posts and the 0.1 s loop

    def __init__(self):
        self.stage, self.vert, self.turn = 0, "H", "S"
        self.ra_t = 0.0
        self.lost_t = None
        self.ack = False
        self.auto = False
        self.auto_reason = None
        self.overridden = False
        self.policy = "backup"
        self.comply = True
        self.reversals = 0
        self.auto_in = None
        self.plan = None
        self.events = deque(maxlen=8)

    def ev(self, text):
        self.events.append("%s %s" % (time.strftime("%H:%M:%S"), text))
        print("[escalation]", text)

    def following(self, pitch, roll):
        if self.vert == "C":
            return pitch >= self.PITCH_OK
        if self.vert == "D":
            return pitch <= -self.PITCH_OK
        if self.turn == "R":
            return roll >= self.ROLL_OK
        if self.turn == "L":
            return roll <= -self.ROLL_OK
        return True

    def update(self, level, tau, vert, turn, pitch, roll, now=None, plan=None):
        now = now or time.time()
        self.plan = plan
        if plan and plan.get("branch"):                 # the calculated escape from the dashboard's CBF-QP
            vert = {"C": "C", "D": "D"}.get(plan["branch"].get("vert"), "H")
            turn = {"L": "L", "R": "R"}.get(plan["branch"].get("turn"), "S")
        if level >= 2 and self.stage < 2:
            self.stage, self.vert, self.turn, self.ra_t = 2, vert, turn, now
            self.ack = self.auto = self.overridden = False
            self.lost_t, self.reversals = None, 0
            self.ev("resolution %s %s" % (vert, turn))
        elif self.stage >= 2 and level >= 2 and plan and (vert != self.vert or turn != self.turn):
            self.vert, self.turn = vert, turn                # the CBF re-plan changed the escape
            self.ev("escape updated %s %s" % (vert, turn))
        elif self.stage >= 2 and level >= 2 and vert in ("C", "D") and vert != self.vert and self.reversals < 1:
            self.vert, self.reversals, self.ra_t, self.ack = vert, self.reversals + 1, now, False
            self.ev("sense reversal")
        if self.stage < 2:
            self.stage = 1 if level == 1 else 0
            self.comply, self.auto_in = True, None
            return
        if level == 0:
            self.ev("clear of conflict" + (", autopilot off, you have control" if self.auto else ""))
            self.stage, self.vert, self.turn, self.auto, self.ack, self.auto_in = 0, "H", "S", False, False, None
            return
        # following = the pilot's own path (from the IMU) already clears the intruder, as calculated by the
        # dashboard; without a fresh plan, fall back to "pitch past 5 degrees in the advised direction"
        self.comply = bool(plan.get("resolving")) if plan and "resolving" in plan else self.following(pitch, roll)
        if self.auto:                                   # the autopilot has the controls: no pilot escalation
            self.lost_t = None
            if self.pushing_against(pitch, roll) or self.policy == "off":
                self.auto, self.overridden = False, True
                self.ev("autopilot off: pilot override")
            self.auto_in = None
            if self.auto:
                return
        if self.comply:
            if self.stage > 2:
                self.ev("pilot following")
            self.stage, self.lost_t = 2, None
            since = 0.0
        else:
            self.lost_t = self.lost_t or now
            since = now - max(self.lost_t, self.ra_t)
            wrong = (self.vert == "C" and pitch < -self.PITCH_OK) or (self.vert == "D" and pitch > self.PITCH_OK)
            if since > self.GRACE and self.stage < 3:
                self.stage = 3
                self.ev("not following")
            if (since > self.SHAKER or wrong) and self.stage < 4:
                self.stage = 4
                self.ev("acknowledge required")
        # ---- automatic avoid
        allowed = self.policy != "off" and not self.overridden
        lsm = plan.get("lastSafe") if plan else None             # seconds until the last safe moment
        due = plan.get("autoDue") if plan else None             # the dashboard says the escape cannot wait
        late = tau is not None and tau <= self.AUTO_TAU             # hard floor straight from the sensor's tau
        if lsm is not None:
            late = late or lsm <= self.LSM_MARGIN or bool(due and "last" in due)
        if allowed and not self.auto:
            if self.policy == "always" or (not self.comply and (self.stage >= 4 or late)):
                self.auto = True
                self.auto_reason = ("auto-fly setting" if self.policy == "always" else
                                    "last safe moment to start the escape" if late else "pilot did not act within 5 s")
                self.ev("automatic avoid engaged (%s)" % self.auto_reason)
        if self.auto and (self.policy == "off" or self.pushing_against(pitch, roll)):
            self.auto, self.overridden = False, True
            self.ev("autopilot off: pilot override")
        if allowed and not self.auto and not self.comply:
            t1 = max(0.0, self.SHAKER - since)
            t2 = max(0.0, (tau - self.AUTO_TAU) / 4.0) if tau is not None else 99.0   # airspace s -> real s (x4)
            if lsm is not None:
                t2 = min(t2, max(0.0, (lsm - self.LSM_MARGIN) / 4.0))
            self.auto_in = round(min(t1, t2), 1)
        else:
            self.auto_in = None

    def pushing_against(self, pitch, roll=0.0):
        return ((self.vert == "C" and pitch <= -self.OVERRIDE) or (self.vert == "D" and pitch >= self.OVERRIDE)
                or (self.turn == "R" and roll <= -20.0) or (self.turn == "L" and roll >= 20.0))

    def do_ack(self):
        if self.stage >= 2:
            self.ack = True
            self.ev("acknowledged")

    def take_control(self):
        if self.auto:
            self.auto, self.overridden = False, True
            self.ev("autopilot off: pilot took control")

    def buzz_why(self):
        if self.auto:
            return "A"
        if self.stage >= 3 and not self.ack:
            return str(self.stage)
        return "-"

    def status(self):
        return {"stage": self.stage, "vert": self.vert, "turn": self.turn, "neg": "-", "ack": int(self.ack),
                "ap": int(self.policy != "off"), "auto": int(self.auto), "auto_reason": self.auto_reason if self.auto else None, "comply": int(self.comply), "peer": 0,
                "peer_eq": 0, "ra_by": "L", "buzz": self.buzz_why(), "auto_in": self.auto_in, "policy": self.policy}


class OwnShip:
    """Our plane in the display, flown in a world frame (east, north, up).

    * Pilot flying: the board's IMU is the control stick. Bank turns the plane (turn rate = g tan(bank) / V)
      and pitch climbs or descends it, each with a small dead band so a resting board flies straight.
    * Automatic avoid: it flies the collision-cone CBF-QP command the dashboard calculates (turn rate and
      vertical speed), re-solved several times a second.
    """

    V = 55.0                      # airspace m/s (about 107 knots)

    def __init__(self, a):
        self.a = a
        self.e = self.n = 0.0
        self.alt = a.alt_ft
        self.psi = 0.0            # deg
        self.omega = 0.0          # deg/s
        self.vs = 0.0             # ft/min
        self.t = time.time()
        self.calm_t = time.time()

    def update(self, imu, level, cmd=None):
        now = time.time()
        dt = min(1.0, now - self.t) * self.a.time_scale
        self.t = now
        roll = imu["roll"] if imu.get("ok") else 0.0
        pitch = imu["pitch"] if imu.get("ok") else 0.0
        if cmd is not None:                                   # automatic avoid: the CBF-QP command
            om_t, vs_t = cmd["omegaDeg"], cmd["vsFpm"]
        else:                                                 # pilot: the IMU is the stick
            bank = roll if abs(roll) > 8 else 0.0
            om_t = math.degrees(9.81 * math.tan(math.radians(max(-45.0, min(45.0, bank)))) / self.V)
            vs_t = max(-2000.0, min(2000.0, (abs(pitch) - 8) * 120.0 * (1 if pitch > 0 else -1))) if abs(pitch) > 8 else 0.0
        self.omega += (om_t - self.omega) * min(1.0, dt / 1.5)
        self.vs += (vs_t - self.vs) * min(1.0, dt / 2.0)
        self.psi = (self.psi + self.omega * dt) % 360.0
        self.e += self.V * math.sin(math.radians(self.psi)) * dt
        self.n += self.V * math.cos(math.radians(self.psi)) * dt
        self.alt += self.vs / 60.0 * dt
        self.alt = max(self.a.alt_ft - 3000.0, min(self.a.alt_ft + 3000.0, self.alt))
        if level == 0 and cmd is None and vs_t == 0.0:        # drift back to the reference altitude between runs
            if now - self.calm_t > 3.0:
                self.alt += (self.a.alt_ft - self.alt) * min(1.0, 0.3 * dt / self.a.time_scale)
        else:
            self.calm_t = now


class IntruderLine:
    """The non-smart plane, seen only by our nose sensor.

    When first detected it is put on our nose line. While our plane flies along that line, its distance
    ALONG the line follows the sensor (range measured from where our plane is along the line), so the
    display shows exactly what the sensor sees, and its speed comes from the sensor's closing rate.
    Once our plane turns away (more than 8 degrees off the line) the other plane keeps the speed it had,
    the way a real aircraft carries on along its path: the sideways separation grows, the alert clears
    when the escape works, and the pass is drawn even after the object leaves the sensor's beam."""

    OFF_LINE_DEG = 8.0
    KEEP_S = 6.0                  # real seconds the other plane stays drawn after the sensor loses it

    def __init__(self):
        self.active = False
        self.free = False

    def update(self, ship, rng, a, dt):
        now = time.time()
        if not self.active:
            if not rng.valid():
                return
            self.active, self.free = True, False
            self.dir = (math.sin(math.radians(ship.psi)), math.cos(math.radians(ship.psi)))
            self.anchor = (ship.e, ship.n)
            self.alt = a.alt_ft if abs(ship.alt - a.alt_ft) < 200 else ship.alt
            self.v = 0.0
            self.seen = now
            self.s = rng.r * a.m_per_cm
        line_psi = math.degrees(math.atan2(self.dir[0], self.dir[1]))
        off = abs((ship.psi - line_psi + 180.0) % 360.0 - 180.0)
        if rng.valid():
            self.seen = now
        elif now - self.seen > (self.KEEP_S if self.free else 0.0):
            self.active = False
            return
        if off > self.OFF_LINE_DEG and not self.free:
            self.free, self.free_t = True, now                            # escape under way: carry on
        if self.free and rng.valid() and self.passed(ship):
            # that encounter is over (the drawn plane is moving away from us) but the sensor still sees
            # something: start again on today's nose line, so the next approach is a new encounter
            if -rng.rdot < a.min_closing_cm_s or now - self.free_t > 3.0:
                self.active = False
                self.update(ship, rng, a, 0.0)
                return
        if self.free:
            self.s -= self.v * dt                                         # v > 0 = coming toward the anchor
        elif rng.valid():
            s_own = (ship.e - self.anchor[0]) * self.dir[0] + (ship.n - self.anchor[1]) * self.dir[1]
            self.s = s_own + rng.r * a.m_per_cm
            closure = -rng.rdot * a.m_per_cm / a.time_scale              # airspace m/s, + = closing
            own_along = ship.V * math.cos(math.radians(ship.psi - line_psi))
            self.v = closure - own_along                                   # its own speed toward us
        self.e = self.anchor[0] + self.dir[0] * self.s
        self.n = self.anchor[1] + self.dir[1] * self.s

    def passed(self, ship):
        """True once the drawn other plane is opening from us (past closest approach)."""
        pe, pn = self.e - ship.e, self.n - ship.n
        ve = -self.dir[0] * self.v - ship.V * math.sin(math.radians(ship.psi))
        vn = -self.dir[1] * self.v - ship.V * math.cos(math.radians(ship.psi))
        return pe * ve + pn * vn > 0.0

    def record(self, rng):
        toward = math.degrees(math.atan2(-self.dir[0], -self.dir[1])) % 360.0
        heading = toward if self.v >= 0 else (toward + 180.0) % 360.0
        return {"id": 2, "role": "other", "label": "Other plane (no system)", "e": round(self.e, 1), "n": round(self.n, 1),
                "alt_ft": round(self.alt), "psi": round(heading, 1), "nose": round(heading, 1), "gs_kt": round(abs(self.v) / KT, 1),
                "vs_fpm": 0.0, "omega": 0.0, "age": 0.05 if self.free else round(time.time() - rng.seen_t, 2), "roll": None, "pitch": None}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default="auto", help="Wi-Fi UDP input; use auto/wifi or a UDP port number (default 4210)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--m-per-cm", type=float, default=25.0, help="meters of airspace per cm of real distance")
    ap.add_argument("--time-scale", type=float, default=4.0, help="how much faster than real time the airspace runs")
    ap.add_argument("--max-cm", type=float, default=300.0,
                    help="ignore readings farther than this (TF-Luna sees up to 8 m; raise it if your space is clear)")
    ap.add_argument("--ta-cm", type=float, default=200.0,
                    help="anything approaching inside this distance gets at least a traffic advisory (0 = off)")
    ap.add_argument("--alt-ft", type=float, default=3000.0, help="altitude both planes start at on the display")
    ap.add_argument("--sense", choices=["climb", "descend"], default="climb", help="resolution direction (no altitude data without a camera)")
    ap.add_argument("--min-closing-cm-s", type=float, default=3.0,
                    help="ignore anything approaching slower than this (cm/s): a still table or wall never alerts")
    ap.add_argument("--no-browser", action="store_true", help="do not open the dashboard automatically")
    ap.add_argument("--no-buzz-escalation", action="store_true",
                    help="leave the buzzer to the ESP32's distance threshold only (no escalation beeps)")
    ap.add_argument("--ta-s", type=float, default=48.0,
                    help="traffic advisory at this tau (airspace s)")
    ap.add_argument("--ra-s", type=float, default=32.0,
                    help="resolution at this tau (airspace s); earlier than TCAS's 25 s so the pilot has about "
                         "2 s of real time before the calculated last safe moment")
    a = ap.parse_args()

    # settings the shared workflow expects (camera-only features switched off)
    a.ft_per_cm = 50.0
    a.pattern_ta, a.pattern_ra, a.pattern_agl_ft, a.runway_hdg = 20.0, 15.0, -1.0, None   # no pattern filter
    a.hmd_ra_m = 1e9                                   # head-on by construction
    a.min_descend_ft = -1e9
    a.min_closure_ms = a.min_closing_cm_s * a.m_per_cm / a.time_scale   # slower than this = not approaching
    a.ta_range_m = a.ta_cm * a.m_per_cm if a.ta_cm > 0 else None        # approaching inside 2 m: traffic
    a.forced_sense = "D" if a.sense == "descend" else "C"
    a.rate_from_geometry = True                         # use RangeTracker's robust closing rate only
    a.realert_holdoff_s = 3.0                           # after "clear of conflict", 3 s before a new alert
    a.alert_confirm_s = 0.5                             # a new alert must last 0.5 s (filters noise blips)

    found = discover(a.serial, "none", 1)
    own = NodeLink("our plane", *found["own"]) if "own" in found else NodeLink("our plane")
    if not own.ser:
        print("Could not open the UDP receiver. Make sure UDP port 4210 is available,")
        print("then run again. The display still opens so you can check it.")
    advisor = workflow.Advisor(a)
    rng = RangeTracker(a)
    ship = OwnShip(a)
    vnode = VirtualNode()
    state = {"report": {"on": False}, "aircraft": [], "buzzer_cm": 50, "buzz_sent": None, "buzz_phase_t": 0.0,
             "plan": None, "plan_t": 0.0}

    intr = IntruderLine()

    def build_aircraft():
        imu = own.imu()
        ac = [{"id": 1, "role": "own", "label": "Our plane", "e": round(ship.e, 1), "n": round(ship.n, 1), "alt_ft": round(ship.alt),
               "psi": round(ship.psi, 1), "nose": round(ship.psi, 1), "gs_kt": round(ship.V / KT, 1), "vs_fpm": round(ship.vs),
               "omega": round(ship.omega, 2), "age": 0.05,
               "roll": imu["roll"] if imu["ok"] else None, "pitch": imu["pitch"] if imu["ok"] else None}]
        if vnode.auto:                                   # show the autopilot flying the escape
            ac[0]["roll"] = round(math.degrees(math.atan(ship.V * math.radians(ship.omega) / 9.81)), 1)
            ac[0]["pitch"] = round(max(-12.0, min(12.0, ship.vs / 150.0)), 1)
        if intr.active:
            ac.append(intr.record(rng))
        return ac

    def drive_buzzer(now):
        """Escalation beeps on the real buzzer through the one command the ESP32 understands:
        BUZZ 800 = sound at any distance, BUZZ 0 = silent. The user's own threshold comes back after."""
        if a.no_buzz_escalation:
            return
        why = vnode.buzz_why()
        if why in ("3", "4", "A"):
            period = {"3": 0.5, "4": 0.3, "A": 1.0}[why]
            on_share = {"3": 0.5, "4": 0.6, "A": 0.2}[why]
            want = "BUZZ 800" if ((now - state["buzz_phase_t"]) % period) < period * on_share else "BUZZ 0"
        else:
            state["buzz_phase_t"] = now
            want = "BUZZ %d" % state["buzzer_cm"]
        if want != state["buzz_sent"]:
            own.write(want)
            state["buzz_sent"] = want

    def loop():
        while True:
            try:
                now = time.time()
                node = own.node_state()
                rng.update(own.us)
                imu = own.imu()
                L = advisor.last or {}
                tau_s = L.get("tau")
                ra = advisor.ra or {}
                plan_now = state["plan"] if state["plan"] and now - state["plan_t"] < 1.5 else None
                vnode.update(advisor.level, tau_s if tau_s is not None and tau_s != float("inf") else None,
                             ra.get("vert", "H"), ra.get("turn", "S"),
                             imu["pitch"] if imu["ok"] else 0.0, imu["roll"] if imu["ok"] else 0.0, now, plan_now)
                node["status"] = vnode.status()
                node["events"] = list(vnode.events)
                ac = build_aircraft()
                node_for_adv = dict(node, us=None)                      # range already comes from the sensor
                rep = advisor.step(ac, node_for_adv)
                rep["us_cm"] = (node.get("us") or {}).get("cm")
                if rep.get("intruder"):
                    rep["intruder"]["range_src"] = "nose sensor"
                    rep["intruder"]["equipped"] = None
                rep["pattern_why"] = "needs the camera"
                rep["radio"]["mode"] = "not used (one board)"
                rep["mode"] = "ultrasonic"
                rep["closing_cm_s"] = round(-rng.rdot, 1) if rng.valid() else None
                plan = state["plan"] if state["plan"] and now - state["plan_t"] < 1.5 else None
                cmd = None
                if vnode.auto:
                    if plan and plan.get("cmd"):
                        cmd = plan["cmd"]                               # the CBF-QP command from the dashboard
                    else:                                               # dashboard closed: simple fallback
                        cmd = {"omegaDeg": 3.0 if vnode.turn != "L" else -3.0, "vsFpm": 1500.0 if vnode.vert != "D" else -1500.0}
                dt_sim = min(1.0, now - ship.t) * a.time_scale
                ship.update(imu, advisor.level, cmd)
                intr.update(ship, rng, a, dt_sim)
                state["report"], state["aircraft"] = rep, ac
                drive_buzzer(now)
            except Exception as e:
                print("[demo]", repr(e))
            time.sleep(0.1)
    threading.Thread(target=loop, daemon=True).start()

    def get_tabletop(q):
        return {"time_scale": a.time_scale, "m_per_cm": a.m_per_cm, "ft_per_cm": a.ft_per_cm, "zeroed": True,
                "fps": 0, "error": "", "mode": "ultrasonic", "aircraft": state["aircraft"], "imu": own.imu(),
                "jet_imu": None, "advisory": state["report"], "raw": raw_with_logic(), "buzzer_cm": state["buzzer_cm"],
                "plan": state["plan"] if state["plan"] and time.time() - state["plan_t"] < 1.5 else None}, 200

    def raw_with_logic():
        r = own.raw_state()
        st = vnode.status()                            # the escalation state now lives on the laptop
        r.update(stage=st["stage"], buzz=st["buzz"], ack=st["ack"], comply=st["comply"],
                 closing_cm_s=round(-rng.rdot, 1) if rng.valid() else None)
        r["events"] = list(vnode.events)
        return r

    def post_intent(d):
        own.set_intent(d)
        return {"ok": True, "sent": own.intent}, 200

    def post_zero(d):
        own.zero_now()                                 # done on the laptop: the ESP32 sketch has no zero command
        return {"ok": True}, 200

    def post_ack(d):
        vnode.do_ack()
        return {"ok": True}, 200

    def post_ap(d):
        vnode.policy = "backup" if d.get("on") else "off"
        if not d.get("on"):
            vnode.take_control()
        return {"ok": True, "policy": vnode.policy}, 200

    def post_autopolicy(d):
        m = d.get("mode")
        if m in ("off", "backup", "always"):
            vnode.policy = m
            if m == "off":
                vnode.take_control()
        return {"ok": True, "policy": vnode.policy}, 200

    def post_plan(d):
        """The dashboard's collision-cone CBF-QP plan for the live encounter (see airspace3d.html)."""
        if isinstance(d, dict) and d.get("branch"):
            state["plan"], state["plan_t"] = d, time.time()
        return {"ok": True}, 200

    def post_override(d):
        vnode.take_control()
        return {"ok": True}, 200

    def post_buzzer(d):
        try:
            cm = int(float(d.get("cm", state["buzzer_cm"])))
        except (TypeError, ValueError):
            return {"ok": False, "error": "cm must be a number"}, 400
        cm = max(0, min(800, cm))
        state["buzzer_cm"] = cm
        own.write("BUZZ %d" % cm)
        state["buzz_sent"] = "BUZZ %d" % cm
        return {"ok": True, "cm": cm}, 200

    server.EXTRA_GET["/api/tabletop"] = get_tabletop
    for path, fn in (("/api/intent", post_intent), ("/api/zero", post_zero), ("/api/ack", post_ack), ("/api/ap", post_ap), ("/api/buzzer", post_buzzer),
                     ("/api/autopolicy", post_autopolicy), ("/api/override", post_override), ("/api/plan", post_plan)):
        server.EXTRA_POST[path] = fn
    server.EXTRA_POST["/api/radio"] = lambda d: ({"ok": True}, 200)
    server.PING_INFO["tabletop"] = True
    server.PING_INFO["mode"] = "ultrasonic"

    def open_browser():
        time.sleep(1.5)
        try:
            import webbrowser
            webbrowser.open("http://localhost:%d" % a.port)
        except Exception:
            pass
    if not a.no_browser:
        threading.Thread(target=open_browser, daemon=True).start()
    print("No camera. ESP32 data arrives over Wi-Fi UDP. Hold the plane level and still for about 2 s after telemetry starts.")
    print("")
    print("   >>> Dashboard: http://localhost:%d  (opening it now; the top bar must say LIVE - BOARD DATA)" % a.port)
    print("")
    try:
        server.serve(a.port, None, "Collision demo (IMU + ultrasonic)", False)
    finally:
        own.write("BUZZ %d" % state["buzzer_cm"])     # leave the board with the user's threshold


if __name__ == "__main__":
    main()
