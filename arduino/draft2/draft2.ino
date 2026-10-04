// draft2.ino : same hardware and same job as draft1.ino, made sturdier.
//
// Same pins as draft1:  MPU6500 SDA A4, SCL A5   |   HC-SR04 TRIG D9, ECHO D10   |   Buzzer D8
// Same behaviour:       attitude for the 3D airliner, distance readout, buzzer + warning under 30 cm
//
// What changed and why
//   * 115200 baud instead of 9600. draft1 sent ~45 characters 25 times a second, more than 9600 baud
//     can carry, so Serial.print blocked and the 3D view lagged further and further behind.
//   * The complementary filter runs HERE with the Arduino's own microsecond clock, at about 150 Hz.
//     The PC no longer guesses dt from when USB packets happen to arrive.
//   * Gyro calibration happens here at power-up and checks the board really is still (retries if not).
//   * I2C timeout: a loose wire can no longer freeze the board. If the MPU stops answering, the sketch
//     keeps the distance warning working and re-connects the MPU every second.
//   * HC-SR04: median of the last 3 echoes, so one bad echo cannot trigger or cancel the warning;
//     buzzer on below 30 cm, off above 33 cm (no chattering at the edge); 25 ms echo timeout (~4 m).
//   * Gyro range +-500 deg/s (draft1 used +-250, which a quick wrist flick can saturate).
//
// Serial out, 115200 (mpu6500_3d.py and tabletop.py both read this):
//   HELLO 1 1 -1          who this is (every 2 s)
//   A <roll> <pitch> <yaw>  degrees, filtered, 50 times a second
//   U <cm>                distance, -1 = no echo, about 16 times a second
//   EV <text>             status messages (calibration, IMU lost / back)
// Serial in:  Z  = zero roll, pitch and yaw (hold the plane level first)

#include <Wire.h>

// ---------------- pins (unchanged from draft1) ----------------
#define MPU_ADDR   0x68
#define BUZZER_PIN 8
#define TRIG_PIN   9
#define ECHO_PIN   10

// ---------------- settings ----------------
#define WARN_CM        30.0   // buzzer on below this (draft1: 30)
#define CLEAR_CM       33.0   // and off again above this
#define BUZZ_HZ        1000   // draft1 tone
#define ECHO_TIMEOUT_US 25000UL
#define PING_EVERY_MS  60     // HC-SR04 needs ~60 ms between pings to avoid old echoes
#define SEND_EVERY_MS  20     // attitude 50 times a second
#define FILTER_ALPHA   0.98   // gyro share in the complementary filter
#define ROLL_SIGN  1          // flip to -1 if the 3D model rolls the wrong way
#define PITCH_SIGN 1
#define YAW_SIGN   1

// ---------------- state ----------------
float roll = 0, pitch = 0, yaw = 0;
float rollZero = 0, pitchZero = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;
bool imuOk = false;
uint8_t imuFails = 0;
unsigned long lastImuUs = 0, lastImuRetryMs = 0;

float dist[3] = { -1, -1, -1 };   // last 3 echoes (cm, -1 = none)
uint8_t distIdx = 0, misses = 0;
float distanceCm = -1;
bool warning = false;

unsigned long lastPingMs = 0, lastSendMs = 0, lastHelloMs = 0;
char line[16];
uint8_t lineLen = 0;

// ================================================================ MPU6500
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

// accelerometer in g, gyro in deg/s
bool readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]) / 16384.0;
  ay = (int16_t)((b[2] << 8) | b[3]) / 16384.0;
  az = (int16_t)((b[4] << 8) | b[5]) / 16384.0;
  gx = (int16_t)((b[8] << 8) | b[9]) / 65.5;
  gy = (int16_t)((b[10] << 8) | b[11]) / 65.5;
  gz = (int16_t)((b[12] << 8) | b[13]) / 65.5;
  return true;
}

// same formulas as the first Python script, so the 3D model moves the same way
float accRoll(float ax, float ay, float az) { return atan2(ay, sqrt(ax * ax + az * az)) * 57.2958; }
float accPitch(float ax, float ay, float az) { return atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958; }

bool initImu() {
  uint8_t who = 0;
  mpuWrite(0x6B, 0x80);            // reset
  delay(100);
  mpuWrite(0x6B, 0x01);            // wake up, PLL clock (steadier than draft1's internal oscillator)
  delay(50);
  if (!mpuRead(0x75, &who, 1)) return false;
  if (!(who == 0x70 || who == 0x68 || who == 0x71 || who == 0x73 || who == 0x74)) {
    Serial.print(F("EV unexpected WHO_AM_I 0x")); Serial.println(who, HEX);
    return false;
  }
  mpuWrite(0x1A, 0x03);            // gyro low-pass ~41 Hz (cuts vibration noise)
  mpuWrite(0x1B, 0x08);            // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);            // accel +-2 g (as draft1)
  if (who != 0x68) mpuWrite(0x1D, 0x03);   // accel low-pass (MPU-6500 family)
  return true;
}

// average the gyro while still; returns false if the board moved during calibration
bool calibrateGyro() {
  float sx = 0, sy = 0, sz = 0, mx = 0, ax, ay, az, gx, gy, gz;
  int n = 0;
  for (int i = 0; i < 300; i++) {
    if (readRaw(ax, ay, az, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz; n++;
      float m = fabs(gx) + fabs(gy) + fabs(gz);
      if (m > mx) mx = m;
    }
    delay(5);
  }
  if (n < 200) return false;
  gxBias = sx / n; gyBias = sy / n; gzBias = sz / n;
  float bias = fabs(gxBias) + fabs(gyBias) + fabs(gzBias);
  if (mx - bias > 15.0) return false;          // it was moving: try again
  if (readRaw(ax, ay, az, gx, gy, gz)) { roll = accRoll(ax, ay, az); pitch = accPitch(ax, ay, az); }
  return true;
}

void startImu() {
  imuOk = initImu();
  if (!imuOk) { Serial.println(F("EV IMU NOT FOUND: check SDA A4, SCL A5, power")); return; }
  for (uint8_t tries = 0; tries < 3; tries++) {
    Serial.println(F("EV CALIBRATING keep still"));
    if (calibrateGyro()) { Serial.println(F("EV CAL DONE")); break; }
    Serial.println(F("EV moved during calibration, again"));
  }
  lastImuUs = micros();
  imuFails = 0;
}

void updateImu() {
  unsigned long nowMs = millis();
  if (!imuOk) {                                 // keep trying to get it back, once a second
    if (nowMs - lastImuRetryMs > 1000) {
      lastImuRetryMs = nowMs;
      if (initImu()) { imuOk = true; imuFails = 0; lastImuUs = micros(); Serial.println(F("EV IMU back")); }
    }
    return;
  }
  float ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) {
    if (++imuFails > 20) { imuOk = false; Serial.println(F("EV IMU lost, retrying")); }
    return;
  }
  imuFails = 0;
  unsigned long now = micros();
  float dt = (now - lastImuUs) / 1e6;
  lastImuUs = now;
  if (dt <= 0 || dt > 0.1) return;              // skip the first sample after a pause
  gx -= gxBias; gy -= gyBias; gz -= gzBias;
  roll = FILTER_ALPHA * (roll + gx * dt) + (1 - FILTER_ALPHA) * accRoll(ax, ay, az);
  pitch = FILTER_ALPHA * (pitch + gy * dt) + (1 - FILTER_ALPHA) * accPitch(ax, ay, az);
  yaw += gz * dt;                               // no magnetometer: yaw slowly drifts, Z resets it
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

// ================================================================ HC-SR04
float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

void updateDistance() {
  unsigned long now = millis();
  if (now - lastPingMs < PING_EVERY_MS) return;
  lastPingMs = now;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  float cm = us ? us * 0.0343 / 2.0 : -1;
  if (cm >= 2 && cm <= 400) {
    dist[distIdx] = cm; distIdx = (distIdx + 1) % 3; misses = 0;
  } else if (++misses >= 3) {                   // three misses in a row: really nothing there
    dist[0] = dist[1] = dist[2] = -1;
  }
  // median of the valid readings
  float v[3]; uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) if (dist[i] > 0) v[n++] = dist[i];
  if (n == 0) distanceCm = -1;
  else if (n == 1) distanceCm = v[0];
  else if (n == 2) distanceCm = (v[0] + v[1]) / 2;
  else distanceCm = median3(v[0], v[1], v[2]);

  // buzzer with hysteresis
  if (!warning && distanceCm > 0 && distanceCm < WARN_CM) warning = true;
  else if (warning && (distanceCm < 0 || distanceCm > CLEAR_CM)) warning = false;
  if (warning) tone(BUZZER_PIN, BUZZ_HZ); else noTone(BUZZER_PIN);

  Serial.print(F("U "));
  Serial.println(distanceCm > 0 ? (int)(distanceCm + 0.5) : -1);
}

// ================================================================ commands
void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen && line[0] == 'Z') { rollZero = roll; pitchZero = pitch; yaw = 0; Serial.println(F("EV ZEROED")); }
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

void hello() { Serial.println(F("HELLO 1 1 -1")); }

// ================================================================ main
void setup() {
  Serial.begin(115200);
  hello();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);   // 3 ms: an I2C glitch can no longer hang the board (AVR core 1.8.3+)
  startImu();
}

void loop() {
  readCommands();
  updateImu();
  updateDistance();
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_EVERY_MS && imuOk) {
    lastSendMs = now;
    Serial.print(F("A "));
    Serial.print(ROLL_SIGN * (roll - rollZero), 1); Serial.print(' ');
    Serial.print(PITCH_SIGN * (pitch - pitchZero), 1); Serial.print(' ');
    Serial.println(YAW_SIGN * yaw, 1);
  }
  if (now - lastHelloMs >= 2000) { lastHelloMs = now; hello(); }
}
