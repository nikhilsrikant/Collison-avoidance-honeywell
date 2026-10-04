#pragma once
#include "Arduino.h"
struct TwoWire {
  uint8_t reg = 0; uint8_t buf[16]; int n = 0, i = 0;
  void begin() {} void setClock(long) {} void setWireTimeout(unsigned long, bool) {}
  void beginTransmission(int) {} 
  void write(uint8_t v) { reg = v; }
  int endTransmission(bool = true) { return 0; }
  int requestFrom(int, int cnt) {
    n = cnt; i = 0;
    if (reg == 0x75) { buf[0] = 0x70; return cnt; }
    if (reg == 0x3B) {
      float th = mockPitch * 3.14159265f / 180.f;
      int16_t ax = (int16_t)(-sinf(th) * 16384), az = (int16_t)(cosf(th) * 16384);
      int16_t v[7] = { ax, 0, az, 0, 0, 0, 0 };
      for (int k = 0; k < 7; k++) { buf[2*k] = (uint8_t)(v[k] >> 8); buf[2*k+1] = (uint8_t)v[k]; }
    }
    return cnt;
  }
  int read() { return i < n ? buf[i++] : 0; }
};
extern TwoWire Wire;
