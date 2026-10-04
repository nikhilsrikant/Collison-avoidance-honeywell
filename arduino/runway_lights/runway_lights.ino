// Runway station lights for the collision avoidance demo (Arduino Uno or Nano).
//
// runway_guard.py sends one letter per line over USB serial (115200 baud):
//   C = runway clear        -> green on
//   A = runway in use       -> red stop lights on (vehicles hold short)
//   O = vehicle on runway   -> red lights blink slowly
//   I = INCURSION           -> red lights flash fast + buzzer
// If nothing arrives for 2 seconds the lights fail safe to steady red with a chirp,
// so a crashed laptop never shows a green light.
//
// The ultrasonic sensor at the hold-short line is a second, independent sensor (Waymo-style redundancy).
// Its distance goes back to the laptop as "D <cm>" ten times a second.
//
// Wiring (see README):
//   D2, D3, D4, D5 -> red LEDs through 220 ohm resistors to GND (runway entrance / stop lights)
//   D6             -> green LED through 220 ohm to GND
//   D8             -> buzzer + (buzzer - to GND). Active or passive buzzer both work.
//   D9             -> HC-SR04 TRIG, D10 -> HC-SR04 ECHO, VCC -> 5V, GND -> GND

const int RED_PINS[] = {2, 3, 4, 5};
const int NUM_RED = 4;
const int GREEN_PIN = 6;
const int BUZZER_PIN = 8;
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

char state = 'X';               // X = no link yet
unsigned long lastCmdMs = 0;
unsigned long lastPingMs = 0;
unsigned long lastChirpMs = 0;

void setReds(bool on) {
  for (int i = 0; i < NUM_RED; i++) digitalWrite(RED_PINS[i], on ? HIGH : LOW);
}

long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000UL);   // about 4 m max, keeps the loop fast
  if (us == 0) return -1;                                   // nothing in range
  return (long)(us / 58UL);
}

void setup() {
  for (int i = 0; i < NUM_RED; i++) pinMode(RED_PINS[i], OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(115200);
  // lamp test so you can see every LED works
  setReds(true); digitalWrite(GREEN_PIN, HIGH); tone(BUZZER_PIN, 2000, 120);
  delay(500);
  setReds(false); digitalWrite(GREEN_PIN, LOW);
  Serial.println("READY");
}

void loop() {
  unsigned long now = millis();

  // 1. read commands from the laptop
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'C' || c == 'A' || c == 'O' || c == 'I') {
      state = c;
      lastCmdMs = now;
    }
  }
  if (now - lastCmdMs > 2000) state = 'X';   // link lost: fail safe

  // 2. drive the lights and buzzer
  bool fast = (now / 90) % 2;     // about 5.5 flashes per second
  bool slow = (now / 500) % 2;
  switch (state) {
    case 'C':
      setReds(false); digitalWrite(GREEN_PIN, HIGH); noTone(BUZZER_PIN);
      break;
    case 'A':
      setReds(true); digitalWrite(GREEN_PIN, LOW); noTone(BUZZER_PIN);
      break;
    case 'O':
      setReds(slow); digitalWrite(GREEN_PIN, LOW); noTone(BUZZER_PIN);
      break;
    case 'I':
      setReds(fast); digitalWrite(GREEN_PIN, LOW);
      if (fast) tone(BUZZER_PIN, 2400); else noTone(BUZZER_PIN);
      break;
    default:  // 'X': no laptop, steady red and a short chirp every 3 s
      setReds(true); digitalWrite(GREEN_PIN, LOW);
      if (now - lastChirpMs > 3000) { tone(BUZZER_PIN, 1500, 60); lastChirpMs = now; }
      break;
  }

  // 3. hold-line sensor, 10 times a second
  if (now - lastPingMs >= 100) {
    lastPingMs = now;
    long cm = readDistanceCm();
    Serial.print("D ");
    Serial.println(cm);
  }
}
