# Collision avoidance prototype for small aircraft

**Objective:** cut midair and runway collisions for aircraft that do not have airliner-grade TCAS (private and training planes, helicopters, and possibly military aircraft) by doing four things a self-driving car does: **perceive** the traffic, **predict** where it will go, **plan** the safest escape, and **act**. Acting means three things:

1. **Tell our pilot**: one clear instruction on screen and by voice.
2. **Tell other pilots**: our wingtip lights show where we are about to go, so even aircraft with no system can move the other way.
3. **Take over if our pilot is late**: auto-avoid flies the escape, then hands control back.

---

## Quick start: one Arduino, no camera (IMU + ultrasonic)

1. Upload `arduino/collision_node/collision_node.ino` to the Uno on the **smart plane** (Arduino IDE: install "Adafruit NeoPixel" first). Keep it still 2 s after power-up, then **close the Serial Monitor**.
2. The **non-smart plane** carries nothing. Our nose sensor (HC-SR04) detects it, the way radar sees an aircraft with no system.
3. On the laptop:
   ```
   python -m pip install pyserial
   python demo.py
   ```
   Open **http://localhost:8765** in Chrome and click the page once (voice). Optional 3D airliner: `python -m pip install vpython`, then `python mpu6500_3d.py`.
4. Point the planes nose to nose about **1.5 m** apart and close at about 10 cm per second (both together). Traffic advisory at roughly 1.4 m, "Climb, climb, turn right" at roughly 90 cm. Then pitch the smart plane's nose up (it counts as following once pitch passes +5°), or ignore it to show the buzzer, "Acknowledge" and the autopilot.

**Check the raw sensors first** (close demo.py and the Serial Monitor): `python sensor_check.py` prints the live distance, pitch, board stage and why the buzzer is on, then lists any problems it found (no echo, distance stuck on the table, IMU not updating, old sketch). The dashboard's left panel, **Board · raw data**, shows the same live numbers and a 10-second distance graph while demo.py runs.

**Aim the HC-SR04 straight ahead, lifted off the table.** Lying flat it can echo off the table or the wires. A still object never sets off an alert now: only something approaching faster than 3 cm/s does (`--min-closing-cm-s`). ACK silences the buzzer for the rest of that encounter.

Without a camera: head-on only (the sensor sees about 15° straight ahead), the resolution is CLIMB by default (`python demo.py --sense descend` for the other way), and there is no traffic-pattern filter. `tabletop.py` is the camera version and needs `opencv-python` and `numpy`.

## Live dashboard with the ESP32 (what changed on the laptop side)

The ESP32 sketch and wiring are unchanged. Everything below runs in `demo.py` and the dashboard.

- **Steady readings.** The TF-Luna distance goes through a 5-reading median, light smoothing and a 1.5 cm hold band; the graph shows the raw reading (thin grey) under the filtered one (green). "Approaching" needs a 2 s straight-line fit that clearly beats the sensor's own scatter, at least 10 cm of movement, and about half a second of agreement, and a new alert must hold for 0.5 s. Resting with +-6 cm of noise gave no alerts in testing; an approach at 10 cm/s alerts at about 1.1 m (traffic) and 75 cm (resolution). Point the TF-Luna at a matte surface: glossy screens and angled glass give the biggest jumps.
- **Zero IMU works.** The board's position when data starts counts as level (so a board resting at an angle no longer flies the display plane upward), and the Zero IMU button re-zeros at any time.
- **Automatic control in live mode.** The escalation and autopilot logic now runs on the laptop, with the same rules as the simulation: resolution, 3 s to start following (pitch past 5 degrees), buzzer, 5 s, "Acknowledge", then automatic avoid. The AUTO-AVOID menu sets the policy: Backup (takes over when you are late, at the calculated last safe moment, with a countdown bar), Always (at once), Off. Take control back with X, the Autopilot button, or by tilting against the escape by 15 degrees. The real buzzer pulses during escalation through the board's existing `BUZZ` command and returns to your threshold afterwards (`--no-buzz-escalation` turns that off).
- **Camera comfort.** Pinch out to zoom in and pinch in to zoom out (laptop trackpad or touch screen), scroll wheel, the + and - buttons or keys, CLOSE for a close-up behind the plane, RESET (or double-click) to start over. Drag to any angle; the dashboard remembers your view and angle next time. In the live chase view the camera keeps the plane just below the instruction panel.

## How the escape is calculated (collision-cone CBF-QP)

Earlier versions picked one of 25 fixed maneuvers (5 headings x 5 climb rates) and flew it at full rate. The escape now comes from a calculation, following the study in `collision_cone_cbf_research_notes.pdf`. The PLANNER menu still offers the old 5x5 grid for comparison.

1. **Collision cone as a safety barrier.** For each intruder, with relative position r and relative velocity w, the barrier is `h = r.w + sqrt(|r|^2 - rho^2) |w|` (Tayal et al., C3BF). `h >= 0` means the relative velocity points outside the cone around the protected zone, so the paths will not meet. Vertical distances are scaled (0.3 NM / 350 ft) so the protected zone is a flat ellipsoid, like TCAS's.
2. **Smallest change that keeps h safe.** Every step a small quadratic program finds the turn rate, vertical acceleration and speed change closest to what the pilot is doing now, subject to `h_dot + gamma h >= 0` for every credible intruder path. The intruder's possible acceleration and the age of its data make the margin bigger (robust term and inflated rho). The QP is solved exactly (an active-set method with enumeration as a backup; a slack variable is logged, and a run that needed slack is never called safe).
3. **Nine pass sides, flown forward.** The QP alone only says "a bit more right"; it cannot choose between passing left or right, over or under. So the planner flies the safety filter forward 40 s for each of nine branches (left / straight / right x climb / level / descend) and scores each one by predicted miss, ground clearance, effort and slack, plus the rules of the air: keep right head-on, give way to traffic on the right (14 CFR 91.113), no descent near the ground, go around on final. A branch that is already being flown gets a bonus, and once the turn is under way the pass side stays fixed unless that side stops working.
4. **One instruction for the pilot.** The best branch becomes the advisory, for example `TURN RIGHT 25 deg BANK TO 132 - HOLD ALTITUDE - 18 S`, with the predicted miss shown in the engineer view.
5. **Last safe moment.** A binary search finds the longest the pilot can keep the present path and still escape with that branch (at least 80 % of the protected zone, no near miss, no terrain conflict). The countdown shows it. Auto avoid takes over at that moment minus 1 s, or after 5 s without a response, whichever comes first.
6. **Auto avoid flies the same QP.** The autopilot re-solves the filter several times a second, so it flies the gentlest command that keeps the barrier, not a fixed full-rate turn.

**Simulation results (the pilot does nothing, auto avoid backup):**

| Scenario | Old 5x5 grid | Collision-cone CBF-QP |
|---|---|---|
| Head-on | safe, 2,036 ft, turned 43 deg | safe, 1,862 ft, turned 37 deg, 169 ft climb |
| Base to final | safe, 3,335 ft | safe, 2,939 ft, turned 25 deg |
| Helicopter crossing | safe, 2,969 ft | safe, 1,985 ft |
| Ridge on the left | safe, 1,831 ft, 540 ft climb | safe, 1,276 ft, 390 ft climb |
| Fire truck on the runway | safe go-around | safe go-around |

Every case stays outside the 0.3 NM protected zone. The CBF-QP passes a little closer because it uses only the maneuver that is needed, which is the point: gentler, more predictable escapes. With a pilot who follows the advisory, or one who first turns the wrong way, all eight test runs ended safe; the only takeover was on the ridge with the wrong-way pilot, at the calculated last safe moment.

**On the live board:** the laptop sends the board's data to the dashboard, the dashboard calculates the plan and posts it back to `demo.py` (`/api/plan`) about 4 times a second. The pilot "flies" by tilting the board (bank turns, pitch climbs). If the pilot does not follow, `demo.py` engages auto avoid at the last safe moment (with 1 s of real time margin for sensor and network lag) or after 5 s, and the display plane flies the CBF-QP command. Bench tests with a simulated ESP32: a pilot who ignores the advisory got auto avoid and passed 1.4 to 1.8 times the protected zone; a pilot who tilted as advised got no takeover and passed about 1.8 to 2 times. In live mode the alert now comes at tau 48 s / 32 s (about 1.1 m and 75 cm at 10 cm/s), a little earlier than before, so a person has about 2 s to react; `--ta-s` and `--ra-s` change it.

This is a research prototype, not certified avionics.

## What is in this folder

| File | What it is |
|---|---|
| `demo.py` | **Main demo without a camera**: one Arduino, IMU + ultrasonic, same display and workflow |
| `sensor_check.py` | Raw check of the board in the terminal: live values plus a list of wiring or sketch problems |
| `nodes.py` | Serial link to the Arduino(s), shared by demo.py and tabletop.py |
| `tabletop.py` | Camera version. Tracks the model planes with an overhead camera, talks to both Arduinos, relays their radio packets, serves the display |
| `workflow.py` | Alerting workflow: fused tau, traffic advisory, resolution sense, nuisance filter, voice |
| `arduino/collision_node/` | **One sketch for both planes** (set `NODE_ID` and `EQUIPPED`): IMU, radio beacon, negotiation, escalation, autopilot gate |
| `airspace3d.html` | The 3D escape display (also opens on its own with 5 scripted scenarios) |
| `arduino/draft2/` | The team's first sketch made sturdier: same parts, same pins |
| `mpu6500_3d.py` | VPython 3D airliner view of either plane, with the live alert |
| `arduino/plane_lights_imu/` | Earlier lights/servo sketch (its pins clash with draft1's wiring; use collision_node) |
| `markers_to_print.pdf` | Tracking markers for the planes (print at 100%) |
| `server.py` | Hub for scripted scenarios, live Phoenix ADS-B and the runway station |
| `runway_guard.py`, `arduino/runway_lights/` | Optional second act: runway incursion station |
| `test_video/` | Practice videos: `tabletop.mp4` (two marked planes head-on) and `table.mp4` (runway station) |

---

## Quick start: the model plane demo

1. **Print** `markers_to_print.pdf` at 100% (check the 10 cm ruler). Tape **marker 1 on our plane** and **marker 2 on the other plane**, flat on top, arrow toward the nose.
2. **Camera**: a 1080p webcam or a phone (Continuity Camera on Mac, DroidCam or Phone Link on Windows), 1.0 to 1.3 m above the table, looking straight down. Even light, no glare.
3. **Our plane's Arduino**: open `arduino/plane_lights_imu/plane_lights_imu.ino`, install the "Adafruit NeoPixel" library (Tools > Manage Libraries), upload. Keep the plane still for 2 s after it powers up (gyro calibration).
4. **Install and run** (Python 3.10+):
   ```
   python -m pip install -r requirements.txt
   python tabletop.py
   ```
   Open **http://localhost:8765** in Chrome or Edge.
5. Put both planes flat on the table and click **Zero table**.
6. **Fly**: start the planes at opposite ends of the table and move them toward each other slowly (about 5 cm per second). Lift them to "fly" at different altitudes.

**No camera yet?** Practice with the included video: `python tabletop.py --video test_video/tabletop.mp4 --serial none`

**Scale** (change with flags): 1 cm on the table = 25 m, 1 cm above the table = 50 ft, time runs 4x faster. So 5 cm per second looks like 60 knots and a 1.6 m table is about 2 nautical miles.
Useful flags: `--camera 1`, `--marker-cm 6`, `--serial COM4`, `--m-per-cm 25`, `--ft-per-cm 50`, `--time-scale 4`.

---

## Our plane: wiring (Arduino Uno R3)

The MPU, HC-SR04 and buzzer stay exactly where the team's first sketch (`draft1.ino`) had them. Everything else is optional and can be added later.

| Part | Pin | Needed for |
|---|---|---|
| MPU6500 | SDA **A4**, SCL **A5**, VCC 5V (3.3V if the board has no regulator), GND | all |
| HC-SR04 | TRIG **D9**, ECHO **D10** (move ECHO to **D3** once a LoRa module is added: D10 becomes SPI select) | all |
| Buzzer | **D8** | all |
| NeoPixel chain, 5 LEDs | data **D6** through 330 Ω. Order: 0 left wingtip, 1 right, 2 tail, 3 top, 4 belly | intent lights |
| Pan servo (turn) | **D7** | autopilot demo |
| Tilt servo (climb/descend) | **A0** | autopilot demo |
| Vibration motor (stick shaker) | **D5** through a 2N2222 (1 kΩ to the base, 1N4148 across the motor) | escalation |
| ACK button | **A2** to GND | escalation |
| Autopilot toggle | **A3** to GND (set `HAS_AP_SWITCH 1`) | optional, the display has a button |
| LoRa RFM95 | NSS **D4**, RST **A1**, DIO0 **D2**, SCK D13, MISO D12, MOSI D11, 3.3 V with level shifting | real radio |
| Servo power | separate 5V (4 x AA or a USB power bank), **ground tied to the Arduino GND** | servos |

**Which sketch?**

| Sketch | When | What it adds over draft1 |
|---|---|---|
| `arduino/draft2/draft2.ino` | Drop-in replacement for `draft1.ino`. Same parts, same pins, same 30 cm buzzer | 115200 baud (no lag), filter on the board, still-check during gyro calibration, I2C timeout + automatic IMU reconnect, 3-echo median and 30/33 cm hysteresis on the buzzer. Works with `mpu6500_3d.py` on its own and with `tabletop.py` |
| `arduino/collision_node/collision_node.ino` | The full system, on both planes | Everything in draft2, plus radio beacon, negotiation, escalation, autopilot gate. With no laptop running it still beeps under 30 cm like draft1 |

`mpu6500_3d.py` (the VPython airliner) now finds the Arduino by itself, reconnects if the cable is pulled, never lags, and, when `tabletop.py` is running, shows the real alert instead of just the distance. `python mpu6500_3d.py --plane jet` shows the jet's attitude.

**Mount the plane on a 2-servo pan-tilt bracket on a handle.** When auto-avoid takes over, the servos visibly bank and pitch the model toward the escape while the person holding it moves it.

**Why not AirTags or the accelerometer for position?** AirTags have no developer API and Find My locations are not real time. Double-integrating the accelerometer drifts within seconds. The overhead camera is real time and accurate to about a centimeter. The MPU-6500 has no magnetometer, so its yaw drifts; the camera supplies heading, and the IMU supplies roll and pitch.

---

## Alerting workflow: two Arduinos, one equipped plane and one private jet

Both planes run the **same sketch**, `arduino/collision_node/collision_node.ino`. Set two lines at the top before uploading:

| Board | `NODE_ID` | `EQUIPPED` | Marker | What it is |
|---|---|---|---|---|
| Our plane | 1 | 1 | 1 | Full system: alerts, resolution, escalation, autopilot |
| Private jet | 2 | 0 | 2 | IMU + ADS-B-style radio beacon + normal lights. Its pilot gets nothing on a screen |

Set `EQUIPPED 1` on the jet too and you get act 2: two equipped aircraft negotiating over the radio.

**Radio.** `RADIO_MODE 0` (default) needs no LoRa module: plug both boards into the laptop and `tabletop.py` carries their packets between the two USB ports. `RADIO_MODE 1` uses real RFM95/SX1276 LoRa modules (915 MHz in the US). The packet is the same 12 bytes either way, so nothing else changes when the modules arrive. The RFM95 is 3.3 V only: on a 5 V Uno it needs a level shifter, or use an ESP32 LoRa board.

**Run it**
```
python tabletop.py                      # finds both boards by the HELLO line each one prints
python tabletop.py --serial COM4 --jet-serial COM5     # or name the ports
python tabletop.py --radio-loss 0.5     # drop half the radio packets (show the tie-break still works)
```

**What happens, step by step**

| Step | Where | Rule |
|---|---|---|
| Fused tau | laptop (`workflow.py`) | Range from the camera, refined by the HC-SR04 when the jet is in its cone. Alpha-beta filter for closing rate. Modified tau = (r² − DMOD²) / (r · closing rate) |
| Traffic advisory | laptop | tau ≤ 40 s and within 850 ft vertically: amber banner, "Traffic, traffic, 12 o'clock" |
| Resolution | laptop + node | tau ≤ 25 s. **Jet not equipped:** the laptop picks the sense that leaves the most room at closest approach, using the jet's IMU pitch from its beacon to see a climb early. Head-on: turn right. One sense reversal allowed. **Jet equipped:** PROPOSE / ACK over the radio, and the fixed rule (lower ID climbs, higher ID descends) decides even if every packet is lost |
| Escalation | node | The IMU checks the pilot: climb = pitch ≥ +5°, descend = pitch ≤ −5°. Not following after 3 s: buzzer and louder voice. After 5 s: stick-shaker motor and ACK required. ACK (button on A2, display button, or K key) silences it for 4 s |
| Autopilot | node | Only if the autopilot is engaged (switch on A3, or the display's Autopilot button). Takes over when the pilot still is not following at stage 4 or tau ≤ 12 s. Hold ACK 1 s, or push against it by 15°, to disconnect |
| Nuisance filter | laptop | Tape marker 4 along the runway, arrow = runway heading. Below 1,000 ft (20 cm), near the runway and within 30° of its axis: TA 20 s, RA 15 s, smaller DMOD |
| Clear | laptop + node | Range opening for 1 s: "Clear of conflict", normal lights, servos centered |

**Wiring:** see the pin table under "Our plane: wiring". Only one HC-SR04 on the table, or the two will hear each other's pings.

**Any other page** can read everything from `GET http://localhost:8765/api/tabletop`: `imu.roll`, `imu.pitch`, `jet_imu`, and the full `advisory` object (level, tau, sense, stage, autopilot, radio). CORS is open, and the port belongs to `tabletop.py`, so close the old serial page first:
```js
setInterval(async () => {
  const d = await (await fetch('http://localhost:8765/api/tabletop')).json();
  plane.rotation.set(d.imu.pitch * Math.PI / 180, 0, -d.imu.roll * Math.PI / 180);
  warning.textContent = d.advisory.banner.state;
}, 50);
```

**Tested here** without hardware. Both sketches compile for the Uno (collision_node: ours 62% flash / 34% RAM, LoRa build 68% / 38%; draft2: 36% / 25%). Built on a laptop against a mock Arduino, collision_node passes 19 checks: escalation timing, ACK, autopilot gate and override, PROPOSE/ACK, the tie-break with 100% packet loss, and the jet raising its own resolution with no laptop. draft2 was checked the same way (attitude at 50 Hz, distance median, buzzer hysteresis), and the 3D viewer survived a cable pull and reconnected on its own.

## How our lights talk to other pilots

| Situation | Lights |
|---|---|
| Normal | red left, green right, white tail, red beacon pulses (standard position and anti-collision lights) |
| Escape, turning | the wingtip on the side we will turn toward **blinks amber** (Blinker style) or changes color (Color style) |
| Escape, climbing / descending | top light flashes white / belly light flashes white |
| Urgency | 2, 4, then 6 flashes per second as the conflict gets closer; 8 when auto-avoid is flying |

**Rule for the other pilot: move away from the flashing side.** Head-on, our right wingtip is on their left, so they turn right and both aircraft diverge (the same as the right-of-way rule, 14 CFR 91.113).

We keep red and green because pilots use them to tell which way an aircraft is pointing, and position lights are required at night. Blinker style is the recommended default; Color style (the team's first idea) stays as an option. New signal colors can be standardized: Mercedes-Benz got approval in California and Nevada in 2023 for turquoise marker lights that show automated driving (SAE J3134).
Electronic channel: ADS-B version 2 can already broadcast an aircraft's selected heading and altitude, so equipped aircraft could receive the same intent digitally.

---

## Auto-avoid when the pilot is late

1. **Traffic** (amber): "Traffic, 2 o'clock, low, 1 mile."
2. **Escape** (red): one instruction, for example "Turn right, climb," and the lights start signaling. A bar counts down **5 seconds** (TCAS assumes pilots respond within about 5 s).
3. **Auto avoid engaged** (magenta): if the pilot has not started a working escape within 5 s, or the calculated last safe moment to start the escape is 1 s away, the system flies the escape. In the demo the servos move and the lights flash 8 times a second.
4. **"Clear of conflict. You have control."** when the other aircraft is moving away. **X** (or a controller bumper) disconnects it early.

Precedents: Airbus aircraft with AP/FD TCAS let the autopilot fly TCAS resolutions (A380 since 2009; also A350, A330, A320 family). The F-16's Auto-GCAS takes over at the last moment, saved its first pilot in November 2014, and lets the pilot override. Auto-ACAS, its midair version, began flight tests in 2016. Small planes with a modern autopilot already have servos that could fly the escape.

Simulation results for the calculated escape are in *How the escape is calculated* above.

The escape warning also comes 4.5 to 13.3 s earlier than straight-line (TCAS-style) prediction in these scenarios.

---

## The display: pilot view and engineer view

- **Pilot view** (default): one instruction in large type, the takeover countdown, one card about the aircraft that matters (distance, altitude, speed, closing speed, time to conflict, where it is going), what our lights are telling others, the 3D view with only the best path, and a small 2D map. When nothing is wrong it shows "Clear" and nothing else.
- **Engineer view** (for judges): all nine escape options in 3D colored safe, tight or unsafe, the escape matrix with the reason for each red option, and the full traffic list.
- **Views**: Chase (behind our plane), Overview (frames both planes, best for an audience), Top.

**Scripted scenarios** (no hardware needed): head-on, base to final, low helicopter crossing, ridge on the left, fire truck on the runway. The **Results** button logs every run for an A/B test: have people fly scenario 4 in "Alert only" and in "3D + escape" and compare closest approach and how often their first move was into a red zone.

---

## Prototype vs. real aircraft

| In the prototype | In a real aircraft |
|---|---|
| Overhead camera + ArUco markers | GPS position + ADS-B Out/In (and TIS-B radar targets) |
| MPU-6500 | attitude and heading sensor (AHRS) |
| Laptop running the display | tablet app (EFB) or panel display |
| NeoPixel lights | LED position and anti-collision lights with an intent pattern |
| Pan-tilt servos | autopilot servos |
| Runway station camera | airport surface sensors |

---

## Judging checklist

| Criterion | How we answer it |
|---|---|
| Objective clearly defined | One-sentence objective (top of this file) + stats: most midairs happen in daylight, good weather, within 5 miles of an airport; DCA 2025 (67 killed) and LaGuardia 2026 |
| Solution addresses the objective | Perceive, predict, plan, act; each piece maps to a collision type (air, low altitude, runway) |
| Conceptual design defined | One block diagram slide: sensors > prediction > planner > display, lights, auto-avoid |
| Prototype or strong visual model | Two live model planes, real lights and servos, live 3D display; scripted scenarios as backup |
| Can be implemented | Uses hardware small planes already carry or can add cheaply: ADS-B receivers ($450 to $1,200), tablets, LED lights, autopilot servos. FAA NORSEE path for non-required safety equipment |
| Technically sound | Physics (closure rates, detection range), TCAS-style timing, latching alerts, right-of-way rule, last-moment auto takeover like Auto-GCAS; simulation table above |
| Usable | Pilot view shows one instruction; A/B test numbers from real people |
| Unique | No system for small aircraft combines escape scoring with terrain, visual intent signaling for non-equipped aircraft, and an auto-avoid backup |
| Provides value | Lives and aircraft; Congress is negotiating an ADS-B In requirement (ROTOR and ALERT Acts), so this is the software that would run on it |
| Scalable | Software + lights + existing autopilots; helicopters (works below 1,000 ft where TCAS goes quiet); military variant with infrared lights visible on night vision goggles |
| Team organized | Roles: hardware lead, software lead, algorithm lead, presenter/research lead; shared task board |
| Inclusive presentation | Every member presents the part they built; Q&A split by topic |
| Prototype represents the solution | The "Prototype vs. real aircraft" table above |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Markers not detected | more light, less glare, lower camera, bigger markers (`--marker-cm` must match the print) |
| Altitude jumps | re-zero with planes flat; keep markers flat on the planes; check `--marker-cm` |
| Planes look too fast or slow | move them about 5 cm/s, or change `--time-scale` |
| IMU: no data | check SDA A4 / SCL A5; open the Serial Monitor at 115200 to see `WHO_AM_I` (0x70 for an MPU-6500), then close it |
| Arduino resets when servos move | power the servos separately and connect the grounds |
| No voice | Chrome or Edge; click the page once first |
| `pip` not found | `python -m pip ...` (Mac: `python3 -m pip ...`) |

## Limits to state in Q&A

- Advisory prototype, not certified avionics. Real escape logic and auto-avoid would go through DO-178C software assurance and build on ACAS X logic.
- ADS-B only sees aircraft that broadcast; ground sensors and future onboard radar cover the rest.
- Lights only help when they can be seen (best at dusk and night); the electronic channel covers daytime.
- Tabletop scale and the simulation's airplane motion are simplified.
