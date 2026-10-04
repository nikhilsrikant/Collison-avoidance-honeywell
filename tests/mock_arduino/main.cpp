#include "Arduino.h"
#include "Wire.h"
MockSerial Serial; TwoWire Wire;
int mockPins[32]; float mockPitch = 0; int mockUs = -1; int mockTone = 0; int mockServo[2] = {90, 90}; int mockServoN = 0;
void MockSerial::pump() {
  char b[256]; int n;
  while ((n = ::read(0, b, sizeof b)) > 0) {
    for (int k = 0; k < n; k++) {
      // test hooks: lines starting with '#' set mock hardware and never reach the sketch
      static std::string cur; char c = b[k];
      if (c == '\n') {
        if (!cur.empty() && cur[0] == '#') {
          if (cur[1] == 'P') mockPitch = atof(cur.c_str() + 2);
          if (cur[1] == 'U') mockUs = atoi(cur.c_str() + 2);
          if (cur[1] == 'K') mockPins[A2] = atoi(cur.c_str() + 2) ? LOW : HIGH;
          if (cur[1] == 'W') mockPins[A3] = atoi(cur.c_str() + 2) ? LOW : HIGH;
          if (cur[1] == 'Q') { printf("Q tone=%d shaker=%d pan=%d tilt=%d\n", mockTone, mockPins[5], mockServo[0], mockServo[1]); fflush(stdout); }
        } else { inbuf += cur; inbuf += '\n'; }
        cur.clear();
      } else cur += c;
    }
  }
}
#include "sketch.cpp"
int main() { for (int i = 0; i < 32; i++) mockPins[i] = HIGH; setup(); for (;;) { loop(); std::this_thread::sleep_for(std::chrono::microseconds(500)); } }
