// Model plane: MPU-6500 attitude + intent lights + auto-avoid servos. Arduino Uno R3.
//
// What it does
//   1. Reads the MPU-6500 (accelerometer + gyroscope) and sends roll, pitch and yaw to the laptop.
//   2. Drives the wingtip, tail and beacon lights. Normally they are standard position lights
//      (red left, green right, white tail). When an escape is active, they show other pilots where
//      THIS plane is about to go.
//   3. If the pilot does not act in time, the laptop engages AUTO-AVOID and two servos visibly steer
//      the model (pan = turn, tilt = climb or descend), the way an autopilot would fly the escape.
//
// Serial, 115200 baud
//   sends     A <roll> <pitch> <yaw>            degrees, 50 times a second
//   receives  N                                 normal position lights, servos centered
//             E <turn> <vert> <hz> <auto>       escape intent. turn = L, R or S (straight)
//                                               vert = C (climb), D (descend) or H (hold altitude)
//                                               hz = flashes per second, 1 to 8 (faster = more urgent)
//                                               auto = 1 when auto-avoid is flying, else 0
//             S B   or   S C                    light style: B = blinker (default), C = color change
//             Z                                 zero roll and pitch (hold the plane level and still)
//   No command for 3 seconds: back to normal lights and centered servos (fail safe).
//
// Light styles
//   B (blinker, recommended): red and green stay red and green (pilots use them to tell which way a
//     plane is facing). The wingtip on the side we will turn toward also flashes AMBER, like a car's
//     turn signal. The top light flashes white for a climb, the belly light for a descent.
//   C (color change): the wingtip on the turn side changes color and flashes (red -> orange on the
//     left, green -> blue on the right). Top and belly lights as above.
//
// Wiring
//   MPU-6500: VCC -> 5V (or 3.3V if your board has no regulator), GND -> GND, SDA -> A4, SCL -> A5
//   NeoPixels (WS2812B), one chain on pin 6 in this order: 0 left wingtip, 1 right wingtip, 2 tail,
//     3 top beacon, 4 belly beacon. 5V and GND to the strip; a 330 ohm resistor in the data line helps.
//   Plain LEDs instead? Set USE_NEOPIXEL to 0: left red D9, right green D10, tail white D11,
//     top D5, belly D3, each through a 220 ohm resistor to GND. Plain LEDs flash but cannot change color.
//   Servos (SG90): pan signal D7, tilt signal D8. Power the servos from a separate 5V source
//     (4 x AA pack or a USB power bank) and connect its GND to the Arduino GND. Servos on the Uno's
//     5V pin can brown out the board.
//
// Library (only for NeoPixels): Arduino IDE > Tools > Manage Libraries > "Adafruit NeoPixel".

#define USE_NEOPIXEL 1
#define NUM_PIXELS 5          // 3 works too (no top/belly): then the tail shows climb (white) or descend (amber)
#define NEO_PIN 6
#define BRIGHTNESS 90         // 0 to 255. Keep it moderate when powered from USB.

#define USE_SERVOS 1
#define PAN_PIN 7
#define TILT_PIN 8
#define PAN_THROW 30          // degrees of pan for a turn
#define TILT_THROW 25         // degrees of tilt for a climb or descent
#define PAN_SIGN 1            // flip to -1 if the model turns the wrong way
#define TILT_SIGN 1           // flip to -1 if the nose goes down for a climb

// Flip a sign here if a reading goes the wrong way for how you mounted the sensor.
#define ROLL_SIGN 1
#define PITCH_SIGN 1
#define YAW_SIGN 1

#include <Wire.h>
#if USE_NEOPIXEL
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel px(NUM_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
#endif
#if USE_SERVOS
#include <Servo.h>
Servo panServo, tiltServo;
#endif

const uint8_t MPU_ADDR = 0x68;   // 0x69 if AD0 is tied high
const int PIN_LEFT = 9, PIN_RIGHT = 10, PIN_TAIL = 11, PIN_TOP = 5, PIN_BELLY = 3;

// attitude state
float roll = 0, pitch = 0, yaw = 0;
float rollZero = 0, pitchZero = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;
unsigned long lastImuUs = 0, lastSendMs = 0, lastServoMs = 0;
bool imuOk = false;

// light and servo state
char mode = 'N';                 // N normal, E escape
char turnDir = 'S', vertDir = 'H';
int flashHz = 3;
bool autoAvoid = false;
char style = 'B';
unsigned long lastCmdMs = 0;
float panPos = 90, tiltPos = 90;
char line[32];
uint8_t lineLen = 0;

// ---------------------------------------------------------------- MPU-6500 (raw registers, no library)
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) return false;
  int16_t rax = (int16_t)((b[0] << 8) | b[1]), ray = (int16_t)((b[2] << 8) | b[3]), raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rgx = (int16_t)((b[8] << 8) | b[9]), rgy = (int16_t)((b[10] << 8) | b[11]), rgz = (int16_t)((b[12] << 8) | b[13]);
  ax = rax / 16384.0; ay = ray / 16384.0; az = raz / 16384.0;   // +-2 g
  gx = rgx / 65.5; gy = rgy / 65.5; gz = rgz / 65.5;             // +-500 deg/s
  return true;
}

void setupImu() {
  uint8_t who = 0;
  mpuWrite(0x6B, 0x80);            // reset
  delay(100);
  mpuWrite(0x6B, 0x01);            // wake up, best clock
  delay(50);
  mpuRead(0x75, &who, 1);
  imuOk = (who == 0x70 || who == 0x68 || who == 0x71 || who == 0x73 || who == 0x74);
  mpuWrite(0x1A, 0x03);            // gyro low-pass about 41 Hz
  mpuWrite(0x1B, 0x08);            // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);            // accel +-2 g
  if (who != 0x68) mpuWrite(0x1D, 0x03);   // accel low-pass (MPU-6500 family only)
  Serial.print("WHO_AM_I 0x");
  Serial.println(who, HEX);
  if (!imuOk) { Serial.println("IMU NOT FOUND: check SDA A4, SCL A5, power"); return; }

  // gyro bias: keep the plane still for 2 seconds after power-up
  Serial.println("CALIBRATING keep still");
  float sx = 0, sy = 0, sz = 0, ax, ay, az, gx, gy, gz;
  int n = 0;
  for (int i = 0; i < 400; i++) {
    if (readRaw(ax, ay, az, gx, gy, gz)) { sx += gx; sy += gy; sz += gz; n++; }
    delay(5);
  }
  if (n > 0) { gxBias = sx / n; gyBias = sy / n; gzBias = sz / n; }
  if (readRaw(ax, ay, az, gx, gy, gz)) {
    roll = atan2(ay, az) * 57.2958;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958;
  }
  Serial.println("CAL DONE");
  lastImuUs = micros();
}

void updateImu() {
  if (!imuOk) return;
  float ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) return;
  unsigned long now = micros();
  float dt = (now - lastImuUs) / 1e6;
  lastImuUs = now;
  if (dt <= 0 || dt > 0.1) return;
  gx -= gxBias; gy -= gyBias; gz -= gzBias;
  float rollAcc = atan2(ay, az) * 57.2958;
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958;
  // complementary filter: gyro for fast motion, accelerometer to stop drift (roll and pitch only)
  roll = 0.98 * (roll + gx * dt) + 0.02 * rollAcc;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * pitchAcc;
  yaw += gz * dt;   // no magnetometer, so yaw slowly drifts; the overhead camera provides heading
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

// ---------------------------------------------------------------- lights
uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
#if USE_NEOPIXEL
  return px.Color(r, g, b);
#else
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
#endif
}

const uint32_t OFF = 0;
#define RED rgb(255, 0, 0)
#define GREEN rgb(0, 255, 0)
#define WHITE rgb(255, 255, 255)
#define AMBER rgb(255, 110, 0)
#define ORANGE rgb(255, 70, 0)
#define BLUE rgb(0, 70, 255)

void showLights(uint32_t left, uint32_t right, uint32_t tail, uint32_t top, uint32_t belly) {
#if USE_NEOPIXEL
  px.setPixelColor(0, left);
  px.setPixelColor(1, right);
  px.setPixelColor(2, tail);
  if (NUM_PIXELS >= 5) { px.setPixelColor(3, top); px.setPixelColor(4, belly); }
  px.show();
#else
  // plain LEDs: on/off only (digitalWrite keeps pins 9 and 10 free of PWM, which the Servo library uses)
  digitalWrite(PIN_LEFT, left ? HIGH : LOW);
  digitalWrite(PIN_RIGHT, right ? HIGH : LOW);
  digitalWrite(PIN_TAIL, tail ? HIGH : LOW);
  digitalWrite(PIN_TOP, top ? HIGH : LOW);
  digitalWrite(PIN_BELLY, belly ? HIGH : LOW);
#endif
}

void updateLights() {
  unsigned long now = millis();
  if (mode == 'E' && now - lastCmdMs > 3000) { mode = 'N'; autoAvoid = false; }   // fail safe
  uint32_t left = RED, right = GREEN, tail = WHITE, top = OFF, belly = OFF;
  bool beacon = (now % 1000) < 90;                              // anti-collision beacon, one short flash a second
  if (mode == 'N') {
    top = beacon ? RED : OFF;
    belly = ((now + 500) % 1000) < 90 ? RED : OFF;
  } else {
    unsigned long period = 1000UL / (unsigned long)constrain(flashHz, 1, 8);
    bool on = (now % period) < period / 2;
    if (style == 'B') {
      if (turnDir == 'L') left = on ? AMBER : RED;
      if (turnDir == 'R') right = on ? AMBER : GREEN;
    } else {
      if (turnDir == 'L') left = on ? ORANGE : OFF;
      if (turnDir == 'R') right = on ? BLUE : OFF;
    }
    if (NUM_PIXELS >= 5) {
      if (vertDir == 'C') top = on ? WHITE : OFF;
      if (vertDir == 'D') belly = on ? WHITE : OFF;
    } else {
      if (vertDir == 'C') tail = on ? WHITE : OFF;
      if (vertDir == 'D') tail = on ? AMBER : OFF;
    }
#if !USE_NEOPIXEL
    if (turnDir == 'L') left = on ? RED : OFF;     // plain LEDs: the turn side simply flashes
    if (turnDir == 'R') right = on ? GREEN : OFF;
#endif
  }
  showLights(left, right, tail, top, belly);
}

// ---------------------------------------------------------------- auto-avoid servos
void updateServos() {
#if USE_SERVOS
  unsigned long now = millis();
  float dt = (now - lastServoMs) / 1000.0;
  lastServoMs = now;
  float panTarget = 90, tiltTarget = 90;
  if (mode == 'E' && autoAvoid) {
    if (turnDir == 'R') panTarget = 90 + PAN_SIGN * PAN_THROW;
    if (turnDir == 'L') panTarget = 90 - PAN_SIGN * PAN_THROW;
    if (vertDir == 'C') tiltTarget = 90 - TILT_SIGN * TILT_THROW;
    if (vertDir == 'D') tiltTarget = 90 + TILT_SIGN * TILT_THROW;
  }
  float step = 120.0 * dt;           // move at most 120 degrees per second, like a real autopilot servo
  panPos += constrain(panTarget - panPos, -step, step);
  tiltPos += constrain(tiltTarget - tiltPos, -step, step);
  panServo.write((int)panPos);
  tiltServo.write((int)tiltPos);
#endif
}

// ---------------------------------------------------------------- commands from the laptop
void handleLine(char *s) {
  if (s[0] == 'N') { mode = 'N'; autoAvoid = false; lastCmdMs = millis(); }
  else if (s[0] == 'E') {
    // format: E <turn> <vert> <hz> <auto>
    char t = 'S', v = 'H';
    int hz = 3, au = 0;
    char *p = s + 1;
    while (*p == ' ') p++;
    if (*p) t = *p++;
    while (*p == ' ') p++;
    if (*p) v = *p++;
    while (*p == ' ') p++;
    if (*p) { hz = atoi(p); while (*p && *p != ' ') p++; }
    while (*p == ' ') p++;
    if (*p) au = atoi(p);
    if (t == 'L' || t == 'R' || t == 'S') turnDir = t;
    if (v == 'C' || v == 'D' || v == 'H') vertDir = v;
    flashHz = constrain(hz, 1, 8);
    autoAvoid = (au == 1);
    mode = 'E';
    lastCmdMs = millis();
  }
  else if (s[0] == 'S') { char c = s[2]; if (c == 'B' || c == 'C') style = c; }
  else if (s[0] == 'Z') { rollZero = roll; pitchZero = pitch; yaw = 0; Serial.println("ZEROED"); }
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { line[lineLen] = 0; handleLine(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
#if USE_NEOPIXEL
  px.begin();
  px.setBrightness(BRIGHTNESS);
#else
  pinMode(PIN_LEFT, OUTPUT); pinMode(PIN_RIGHT, OUTPUT); pinMode(PIN_TAIL, OUTPUT);
  pinMode(PIN_TOP, OUTPUT); pinMode(PIN_BELLY, OUTPUT);
#endif
#if USE_SERVOS
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(90);
  tiltServo.write(90);
#endif
  showLights(WHITE, WHITE, WHITE, WHITE, WHITE);   // lamp test
  delay(400);
  showLights(RED, GREEN, WHITE, OFF, OFF);
  setupImu();
  lastServoMs = millis();
  Serial.println("READY");
}

void loop() {
  readCommands();
  updateImu();
  unsigned long now = millis();
  if (now - lastSendMs >= 20) {        // 50 Hz to the laptop
    lastSendMs = now;
    Serial.print("A ");
    Serial.print(ROLL_SIGN * (roll - rollZero), 1);
    Serial.print(' ');
    Serial.print(PITCH_SIGN * (pitch - pitchZero), 1);
    Serial.print(' ');
    Serial.println(YAW_SIGN * yaw, 1);
    updateLights();
    updateServos();
  }
}
