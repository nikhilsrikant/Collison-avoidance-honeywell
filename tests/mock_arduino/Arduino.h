#pragma once
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>
using std::min; using std::max;
typedef bool boolean; typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HEX 16
#define DEC 10
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define F(x) (x)
template<class T, class L, class H> auto constrain(T x, L lo, H hi) -> T { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }
static auto T0 = std::chrono::steady_clock::now();
inline unsigned long millis() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - T0).count(); }
inline unsigned long micros() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - T0).count(); }
inline void delay(unsigned long ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
inline void delayMicroseconds(unsigned int) {}
extern int mockPins[32]; extern float mockPitch; extern int mockUs;
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { mockPins[p] = v; }
inline int digitalRead(int p) { return mockPins[p]; }
inline unsigned long pulseIn(int, int, unsigned long) { return mockUs > 0 ? mockUs * 58UL : 0; }
extern int mockTone;
inline void tone(int, unsigned int) { mockTone = 1; }
inline void noTone(int) { mockTone = 0; }
struct MockSerial {
  std::string inbuf;
  void begin(long) { fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK); setvbuf(stdout, NULL, _IOLBF, 0); }
  void pump();
  int available() { pump(); return (int)inbuf.size(); }
  int read() { if (inbuf.empty()) return -1; int c = (unsigned char)inbuf[0]; inbuf.erase(0, 1); return c; }
  void print(const char *s) { fputs(s, stdout); }
  void print(char c) { putchar(c); }
  void print(int v, int base = DEC) { if (base == HEX) printf("%X", v); else printf("%d", v); }
  void print(unsigned int v, int base = DEC) { if (base == HEX) printf("%X", v); else printf("%u", v); }
  void print(long v) { printf("%ld", v); }
  void print(unsigned long v) { printf("%lu", v); }
  void print(uint8_t v, int base = DEC) { if (base == HEX) printf("%X", v); else printf("%u", v); }
  void print(int8_t v) { printf("%d", v); }
  void print(double v, int d = 2) { printf("%.*f", d, v); }
  void print(bool v) { printf("%d", v ? 1 : 0); }
  template<class T> void println(T v) { print(v); putchar('\n'); fflush(stdout); }
  template<class T> void println(T v, int b) { print(v, b); putchar('\n'); fflush(stdout); }
  void println() { putchar('\n'); fflush(stdout); }
};
extern MockSerial Serial;
