"""Alerting workflow: fused tau -> traffic advisory -> resolution -> escalation -> autopilot -> clear.

Runs on the laptop inside tabletop.py, 10 times a second. The safety-critical parts that must keep
working if the laptop stops (tie-break, escalation timers, ACK, autopilot gate, override) run on our
plane's Arduino (arduino/collision_node). This module decides WHEN to alert and, against an
unequipped aircraft, WHICH WAY to go; the node does the rest and reports back on its S line.

Units: everything here is in the scaled airspace that tabletop.py reports (meters, feet, knots),
and tau is in airspace seconds (real seconds x time_scale).

Sources fused for each intruder
  * camera (stand-in for ADS-B / GPS): position, altitude, ground speed, vertical speed
  * radio beacon from the other aircraft (stand-in for ADS-B Out / LoRa): equipped flag and its IMU
    pitch, which shows a climb or descent before the camera sees the altitude change
  * HC-SR04 on our nose: direct range when the other aircraft is inside the sensor cone
"""
import math
import time

KT = 0.514444
NM = 1852.0
FPM = 1.0 / 60.0          # ft/min -> ft/s


def ang_diff(a, b):
    d = (a - b) % 360.0
    return d - 360.0 if d > 180 else d


class AlphaBeta:
    """Range and range-rate filter (the radar tracker's classic)."""

    def __init__(self, r, rdot=0.0, alpha=0.5, beta=0.2):
        self.r, self.rdot, self.a, self.b = r, rdot, alpha, beta

    def update(self, z, dt):
        if dt <= 0:
            return
        pred = self.r + self.rdot * dt
        res = z - pred
        self.r = pred + self.a * res
        self.rdot = self.rdot + self.b * res / dt


class Advisor:
    def __init__(self, args):
        self.a = args
        self.tracks = {}            # intruder id -> {"f": AlphaBeta, "t": sim time, ...}
        self.level = 0
        self.level_t = 0.0          # real time the level was entered
        self.diverge_t = None
        self.ra = None              # active resolution: {"vert","turn","id","t","reversals","why"}
        self.clear_t = 0.0          # when the last resolution cleared
        self.want_t = None          # since when a new alert has been wanted
        self.pattern = False
        self._on_since = None
        self._off_since = None
        self.say_seq = 0
        self.say = {"id": 0, "text": "", "prio": 0, "loud": False}
        self.last_stage = 0
        self.last_auto = 0
        self.last_esc_say = 0.0
        self.last = {}

    # ------------------------------------------------------------------ helpers
    def announce(self, text, prio=2, loud=False):
        self.say_seq += 1
        self.say = {"id": self.say_seq, "text": text, "prio": prio, "loud": loud}

    @staticmethod
    def clock(own, intr):
        brg = math.degrees(math.atan2(intr["e"] - own["e"], intr["n"] - own["n"]))
        rel = ang_diff(brg, own["psi"])
        c = int(round(rel / 30.0)) % 12
        return 12 if c == 0 else c, rel

    def thresholds(self):
        if self.pattern:
            return {"ta": self.a.pattern_ta, "ra": self.a.pattern_ra, "dmod_nm": 0.2, "zthr_ta": 850, "zthr_ra": 600}
        return {"ta": self.a.ta_s, "ra": self.a.ra_s, "dmod_nm": 0.3, "zthr_ta": 850, "zthr_ra": 700}

    # ------------------------------------------------------------------ nuisance filter: in the traffic pattern?
    def update_pattern(self, own, runway, now):
        why = ""
        cand = False
        hdg = None
        if runway is not None:
            hdg = runway["nose"]
        elif self.a.runway_hdg is not None:
            hdg = self.a.runway_hdg
        if hdg is not None and own is not None:
            low = own["alt_ft"] < self.a.pattern_agl_ft
            near = True
            if runway is not None:
                near = math.hypot(own["e"] - runway["e"], own["n"] - runway["n"]) < 2.0 * NM
            off = min(abs(ang_diff(own["psi"], hdg)), abs(ang_diff(own["psi"], hdg + 180.0)))
            aligned = off < 30.0
            cand = low and near and aligned
            why = "%d ft above field, %s runway, %s (%.0f deg off axis)" % (
                own["alt_ft"], "near" if near else "far from", "aligned" if aligned else "not aligned", off)
        else:
            why = "no runway marker in view"
        # hysteresis: 1 s to enter, 3 s to leave, so it does not flicker
        if cand:
            self._off_since = None
            self._on_since = self._on_since or now
            if now - self._on_since > 1.0:
                self.pattern = True
        else:
            self._on_since = None
            self._off_since = self._off_since or now
            if now - self._off_since > 3.0:
                self.pattern = False
        return why

    # ------------------------------------------------------------------ geometry + fusion for one intruder
    def assess(self, own, intr, us, beacons, now, thr):
        a = self.a
        de, dn = intr["e"] - own["e"], intr["n"] - own["n"]
        dalt = intr["alt_ft"] - own["alt_ft"]
        r_cam = math.hypot(de, dn)

        # HC-SR04: only trust it when the other aircraft is inside the cone and agrees with the camera
        src = "camera"
        r_meas = r_cam
        if us and us.get("cm") is not None and us["cm"] > 3 and us["age"] < 0.3:
            h_cm = r_cam / a.m_per_cm
            dz_cm = dalt / a.ft_per_cm
            brg = math.degrees(math.atan2(de, dn))
            off_nose = abs(ang_diff(brg, own["nose"]))
            elev = math.degrees(math.atan2(abs(dz_cm), max(h_cm, 1e-3)))
            if off_nose < 20 and elev < 20:
                us_h = math.sqrt(max(0.0, us["cm"] ** 2 - dz_cm ** 2)) * a.m_per_cm
                if abs(us_h - r_cam) < max(0.35 * r_cam, 10 * a.m_per_cm):
                    r_meas = 0.3 * r_cam + 0.7 * us_h
                    src = "camera + ultrasonic"

        # relative velocity from the camera tracks
        def vel(p):
            v = p["gs_kt"] * KT
            return v * math.sin(math.radians(p["psi"])), v * math.cos(math.radians(p["psi"]))
        ove, ovn = vel(own)
        ive, ivn = vel(intr)
        dve, dvn = ive - ove, ivn - ovn
        geo_rdot = (de * dve + dn * dvn) / max(r_cam, 1.0)

        tr = self.tracks.get(intr["id"])
        t_sim = now * a.time_scale
        if tr is None or t_sim - tr["t"] > 2.0 * a.time_scale:
            tr = {"f": AlphaBeta(r_meas, geo_rdot), "t": t_sim}
            self.tracks[intr["id"]] = tr
        else:
            tr["f"].update(r_meas, t_sim - tr["t"])
            tr["t"] = t_sim
        f = tr["f"]
        # blend the filtered range-rate with geometry so a hand jitter does not fake a closure
        rdot = 0.6 * f.rdot + 0.4 * geo_rdot
        r = max(f.r, 0.0)
        if getattr(a, "rate_from_geometry", False):     # demo.py: range rate already measured robustly
            rdot, r = geo_rdot, max(r_meas, 0.0)
        closure = -rdot                                  # m/s, positive = getting closer

        # modified tau (TCAS): keeps slow, close encounters from slipping under the threshold
        dmod = thr["dmod_nm"] * NM
        min_close = getattr(self.a, "min_closure_ms", 10.0)
        if closure <= min_close:
            tau = math.inf                               # not approaching: never an alert, however close
        elif r < dmod:
            tau = 0.0
        else:
            tau = max(0.0, (r * r - dmod * dmod) / (r * closure))

        # vertical: ignore traffic that is well above or below and not closing vertically
        ivs = intr["vs_fpm"]
        trend, trend_src = 0, "camera"
        bc = beacons.get(intr["id"])
        equipped = None
        if bc and now - bc["t"] < 1.0:
            equipped = bool(bc["eq"])
            if abs(bc["pitch"]) >= 5:
                trend, trend_src = (1 if bc["pitch"] > 0 else -1), "its IMU (radio)"
                ivs = trend * max(abs(ivs), 800.0)       # its IMU shows the climb before the camera does
        if trend == 0 and abs(ivs) > 300:
            trend = 1 if ivs > 0 else -1
        dvs = ivs - own["vs_fpm"]                         # fpm, + = intruder rising relative to us
        v_closing = dalt * dvs < 0
        t_coalt = abs(dalt) / (abs(dvs) * FPM) if v_closing and abs(dvs) > 50 else math.inf

        # predicted horizontal miss distance at closest approach
        dv2 = dve * dve + dvn * dvn
        tcpa = -(de * dve + dn * dvn) / dv2 if dv2 > 1e-6 else 0.0
        tcpa = max(0.0, tcpa)
        hmd = math.hypot(de + dve * tcpa, dn + dvn * tcpa)

        clock, rel_brg = self.clock(own, intr)
        return {
            "id": intr["id"], "label": intr.get("label", "Plane %d" % intr["id"]), "r": r, "r_cam": r_cam,
            "range_src": src, "closure": closure, "tau": tau, "dalt": dalt, "ivs": ivs, "trend": trend,
            "trend_src": trend_src, "t_coalt": t_coalt, "hmd": hmd, "clock": clock, "rel_brg": rel_brg,
            "equipped": equipped, "intr": intr,
        }

    def threat_level(self, x, thr):
        def vert_ok(zthr, tthr):
            return abs(x["dalt"]) < zthr or x["t_coalt"] < tthr
        lvl = 0
        if x["tau"] <= thr["ta"] and vert_ok(thr["zthr_ta"], thr["ta"]):
            lvl = 1
        ta_r = getattr(self.a, "ta_range_m", None)        # demo.py: anything approaching inside 2 m is traffic
        if ta_r and x["r"] <= ta_r and math.isfinite(x["tau"]) and vert_ok(thr["zthr_ta"], thr["ta"]):
            lvl = 1
        if x["tau"] <= thr["ra"] and vert_ok(thr["zthr_ra"], thr["ra"]) and x["hmd"] < self.a.hmd_ra_m:
            lvl = 2
        return lvl

    # ------------------------------------------------------------------ resolution sense vs an unequipped aircraft
    def choose_sense(self, own, x):
        """Pick the vertical sense that leaves the most room at closest approach.
        Our response model: 5 s at today's vertical speed, then 1,500 ft/min in the chosen direction."""
        tau = x["tau"] if math.isfinite(x["tau"]) else 30.0
        tau = max(tau, 8.0)
        intr_alt = x["intr"]["alt_ft"] + x["ivs"] * FPM * tau
        best, seps = None, {}
        for s, rate in (("C", 1500.0), ("D", -1500.0)):
            delay = min(5.0, tau)
            own_alt = own["alt_ft"] + own["vs_fpm"] * FPM * delay + rate * FPM * (tau - delay)
            seps[s] = abs(own_alt - intr_alt)
        forced = getattr(self.a, "forced_sense", None)       # no altitude data (demo.py): fixed direction
        if forced in ("C", "D"):
            best, why = forced, "no altitude data without a camera: %s by default" % ("climb" if forced == "C" else "descend")
        elif own["alt_ft"] < self.a.min_descend_ft:
            best, why = "C", "too low to descend"
        elif abs(seps["C"] - seps["D"]) < 150:
            # nearly equal: do not cross the other aircraft's altitude, move away from its trend
            if x["trend"] > 0:
                best, why = "D", "it is climbing"
            elif x["trend"] < 0:
                best, why = "C", "it is descending"
            else:
                if abs(x["dalt"]) < 100:
                    best, why = "C", "same altitude, climb by default"
                else:
                    best, why = ("C", "we are higher") if x["dalt"] < 0 else ("D", "we are lower")
        else:
            best = "C" if seps["C"] > seps["D"] else "D"
            why = "%s leaves %d ft at closest approach" % ("climbing" if best == "C" else "descending", seps[best])
        # lateral: head-on both turn right (14 CFR 91.113); otherwise turn away from it
        rb = x["rel_brg"]
        if abs(rb) < 30:
            turn = "R"
        else:
            turn = "L" if rb > 0 else "R"
        return best, turn, why, seps

    # ------------------------------------------------------------------ main step
    def step(self, aircraft, node, now=None):
        """aircraft: tabletop snapshot list. node: dict from our plane's Arduino (or None)."""
        now = now or time.time()
        own = next((p for p in aircraft if p["role"] == "own" and p["age"] < 1.0), None)
        runway = next((p for p in aircraft if p["role"] == "runway" and p["age"] < 2.0), None)
        others = [p for p in aircraft if p["role"] == "other" and p["age"] < 1.0]
        pattern_why = self.update_pattern(own, runway, now)
        thr = self.thresholds()
        node = node or {}
        beacons = node.get("peers", {})
        st = node.get("status")

        assessed, top = [], None
        if own is not None:
            for p in others:
                x = self.assess(own, p, node.get("us"), beacons, now, thr)
                x["level"] = self.threat_level(x, thr)
                assessed.append(x)
            if assessed:
                top = min(assessed, key=lambda x: (-x["level"], x["tau"]))
        for k in list(self.tracks):
            if k not in {p["id"] for p in others}:
                del self.tracks[k]

        # ---- level with latching (alerts never flicker on and off)
        prev = self.level
        want = top["level"] if top else 0
        if self.level == 2:
            lost = top is None
            diverging = top is not None and top["closure"] < getattr(self.a, "min_closure_ms", 10.0)
            if diverging or lost:
                self.diverge_t = self.diverge_t or now
            else:
                self.diverge_t = None
            if self.diverge_t and now - self.diverge_t > 1.5:      # 1.5 s not closing (or gone): clear
                self.level = 0
        elif self.level == 1:
            if want == 2:
                self.level = 2
            elif want == 0 and now - self.level_t > 3.0 and (top is None or top["tau"] > thr["ta"] * 1.25):
                self.level = 0                          # hysteresis: no flicker at the 40 s edge
        else:
            hold = getattr(self.a, "realert_holdoff_s", 0.0)
            if prev == 0 and self.clear_t and now - self.clear_t < hold:
                want = 0                               # just cleared: give it a moment before alerting again
            confirm = getattr(self.a, "alert_confirm_s", 0.0)
            if want > 0 and confirm > 0:               # a new alert must hold for a moment (noise makes blips)
                self.want_t = self.want_t or now
                if now - self.want_t < confirm:
                    want = 0
            else:
                self.want_t = None
            self.level = want
        if self.level == 0 and prev == 2:
            self.clear_t = now
        if self.level != prev:
            self.level_t = now
            self.diverge_t = None

        # ---- resolution sense
        peer_eq = bool(st and st.get("peer") and st.get("peer_eq"))
        if self.level == 2 and top is not None:
            sense, turn, why, seps = self.choose_sense(own, top)
            if self.ra is None or self.ra["id"] != top["id"]:
                self.ra = {"vert": sense, "turn": turn, "id": top["id"], "t": now, "reversals": 0, "why": why, "seps": seps}
            else:
                cur = self.ra["vert"]
                if (sense != cur and self.ra["reversals"] < 1 and now - self.ra["t"] > 2.0
                        and seps[cur] < 300 and seps[sense] > seps[cur] + 300 and not peer_eq):
                    self.ra.update(vert=sense, reversals=self.ra["reversals"] + 1, why="reversal: " + why, t=now)
                    self.announce(("Climb, climb NOW. " if sense == "C" else "Descend, descend NOW. "), 4, True)
                self.ra["seps"] = seps          # the turn stays as issued; only the vertical sense may reverse
        elif self.level < 2:
            self.ra = None

        # final sense: what our node actually flies (it applies the tie-break with an equipped peer)
        if st and st.get("stage", 0) >= 2:
            vert, turn, by = st["vert"], st["turn"], ("tie-break" if st.get("neg") in ("P", "A", "U", "R") else "unilateral")
        elif self.ra:
            vert, turn, by = self.ra["vert"], self.ra["turn"], "unilateral"
        else:
            vert, turn, by = "H", "S", ""

        # ---- voice on transitions
        stage = st["stage"] if st else (2 if self.level == 2 else self.level)
        if self.level == 1 and prev == 0 and top:
            hi = "high" if top["dalt"] > 300 else "low" if top["dalt"] < -300 else "level"
            self.announce("Traffic, traffic. %d o'clock, %s." % (top["clock"], hi), 2)
        if self.level == 2 and prev < 2:
            words = {"C": "Climb, climb.", "D": "Descend, descend.", "H": "Monitor vertical speed."}[vert]
            words += {"R": " Turn right.", "L": " Turn left.", "S": ""}[turn]
            self.announce(words, 3)
        if self.level == 0 and prev == 2:
            self.announce("Clear of conflict.", 1)
        if st:
            if stage >= 3 and (self.last_stage < 3 or now - self.last_esc_say > 3.0) and not st.get("ack"):
                self.last_esc_say = now
                base = "Climb now, climb now." if vert == "C" else "Descend now, descend now." if vert == "D" else "Turn now."
                self.announce(base + (" Acknowledge." if stage >= 4 else ""), 4, True)
            if st.get("auto") and not self.last_auto:
                self.announce("Autopilot avoiding.", 4)
            if self.last_auto and not st.get("auto") and self.level == 2:
                self.announce("Autopilot off. You have control.", 3)
            self.last_auto = st.get("auto", 0)
        self.last_stage = stage

        # ---- tau for the node and the display (airspace seconds)
        tau_s = top["tau"] if top else math.inf
        self.last = {"level": self.level, "tau": tau_s, "vert": vert if self.level == 2 else "H",
                     "turn": turn if self.level == 2 else "S"}
        return self.report(own, top, thr, pattern_why, vert, turn, by, stage, st, node, assessed)

    def node_command(self):
        """Line for our plane's Arduino: T <level> <tau_s> <vert> <turn>."""
        L = self.last or {"level": 0, "tau": math.inf, "vert": "H", "turn": "S"}
        tau = 255 if not math.isfinite(L["tau"]) else int(min(255, max(0, round(L["tau"]))))
        vert = self.ra["vert"] if (self.ra and L["level"] == 2) else "H"
        turn = self.ra["turn"] if (self.ra and L["level"] == 2) else "S"
        return "T %d %d %s %s" % (L["level"], tau, vert, turn)

    # ------------------------------------------------------------------ what the display shows
    def report(self, own, top, thr, pattern_why, vert, turn, by, stage, st, node, assessed):
        V = {"C": "CLIMB", "D": "DESCEND", "H": "HOLD ALTITUDE"}
        T = {"R": "turn RIGHT", "L": "turn LEFT", "S": ""}
        cls, state, msg, sub = "clear", "Clear", "No conflicts", "Watching %d aircraft" % len(assessed)
        if own is None:
            cls, state, msg, sub = "traffic", "Camera", "Our plane is not in view", "Keep marker 1 face-up under the camera."
        elif self.level == 1 and top:
            hi = "high" if top["dalt"] > 300 else "low" if top["dalt"] < -300 else "level"
            cls, state = "traffic", "Traffic advisory"
            msg = "Traffic %d o'clock, %s, %.1f nm" % (top["clock"], hi, top["r"] / NM)
            sub = "%s · %s" % (top["label"], "tau %.0f s" % top["tau"] if math.isfinite(top["tau"]) else "not closing")
        elif self.level == 2 and top:
            cls, state = "escape", "Resolution"
            msg = "<em>%s</em>%s" % (V[vert], (" · <em>%s</em>" % T[turn]) if T[turn] else "")
            if by == "tie-break":
                sub = "Coordinated with %s over the radio: lower ID climbs, higher ID descends." % top["label"]
            else:
                sub = "%s has no system: we maneuver alone (%s). Our lights show the turn." % (
                    top["label"], (self.ra or {}).get("why", ""))
            if st and stage >= 3:
                state = "Not following" if stage == 3 else "ACKNOWLEDGE"
                sub = "The IMU does not see the %s. %s" % (V[vert].lower(), "Press ACK." if stage >= 4 else "")
            if st and st.get("auto"):
                cls, state = "auto", "Autopilot avoiding"
                sub = "Autopilot is flying the escape. Hold ACK 1 s or push against it to take over."
        if self.level == 2 and not top and own is not None:     # traffic just passed or dropped out of view
            cls, state = "escape", "Resolution"
            msg = "<em>%s</em>%s" % (V[vert], (" · <em>%s</em>" % T[turn]) if T[turn] else "")
            sub = "Traffic no longer detected, confirming clear of conflict..."
        tau = top["tau"] if top else math.inf

        def r1(v, k=1):
            return None if v is None or not math.isfinite(v) else round(v, k)
        radio = {"mode": node.get("radio"), "peers": {}, "rx": node.get("rx", 0), "loss": node.get("loss", 0.0)}
        for pid, b in node.get("peers", {}).items():
            radio["peers"][pid] = {"eq": b["eq"], "pitch": b["pitch"], "rssi": b["rssi"], "age": round(time.time() - b["t"], 1)}
        return {
            "on": True,
            "level": self.level, "level_name": ["clear", "traffic", "resolution"][self.level],
            "stage": stage,
            "tau": r1(tau), "thresholds": thr,
            "pattern": self.pattern, "pattern_why": pattern_why,
            "intruder": None if not top else {
                "id": top["id"], "label": top["label"], "equipped": top["equipped"],
                "range_nm": r1(top["r"] / NM, 2), "range_src": top["range_src"],
                "closure_kt": r1(top["closure"] / KT, 0), "dalt_ft": r1(top["dalt"], 0),
                "trend": top["trend"], "trend_src": top["trend_src"], "clock": top["clock"],
                "hmd_nm": r1(top["hmd"] / NM, 2),
            },
            "sense": {"vert": vert, "turn": turn, "by": by, "neg": st.get("neg") if st else None,
                      "why": (self.ra or {}).get("why", "")},
            "node": st, "node_connected": bool(st),
            "us_cm": (node.get("us") or {}).get("cm"),
            "radio": radio,
            "events": node.get("events", []),
            "banner": {"cls": cls, "state": state, "msg": msg, "sub": sub},
            "say": self.say,
        }
