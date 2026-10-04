#pragma once
#include "Arduino.h"
extern int mockServo[2]; extern int mockServoN;
struct Servo { int idx = -1; void attach(int p) { idx = p == 7 ? 0 : 1; } void write(int v) { if (idx >= 0) mockServo[idx] = v; } };
