#pragma once
#include "Arduino.h"
#define NEO_GRB 0
#define NEO_KHZ800 0
struct Adafruit_NeoPixel { uint32_t px[8];
  Adafruit_NeoPixel(int, int, int) {} void begin() {} void setBrightness(int) {}
  uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return ((uint32_t)r << 16) | (g << 8) | b; }
  void setPixelColor(int i, uint32_t c) { px[i] = c; } void show() {} };
