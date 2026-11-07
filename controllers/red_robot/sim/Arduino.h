// Minimal Arduino / ESP32 stand-in so the Red Robot sketch compiles as a Webots
// controller. Only what a typical Red Robot sketch needs is provided; if your
// code uses something missing, add it here.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

template <class A, class B> inline typename std::common_type<A, B>::type max(A a, B b) { return a > b ? a : b; }
template <class A, class B> inline typename std::common_type<A, B>::type min(A a, B b) { return a < b ? a : b; }
using std::abs;

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// timing (simulated time: delay() advances the simulation)
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long millis();
unsigned long micros();
inline void yield() {}

// GPIO: there is no real hardware in the sim. These are no-ops that warn once.
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void analogWrite(int pin, int value);

inline long random(long howbig) { return howbig <= 0 ? 0 : std::rand() % howbig; }
inline long random(long howsmall, long howbig) { return howsmall >= howbig ? howsmall : howsmall + random(howbig - howsmall); }
inline void randomSeed(unsigned long seed) { std::srand((unsigned)seed); }

// FreeRTOS bits that sometimes appear in sketches
#define pdMS_TO_TICKS(ms) (ms)
#define portMAX_DELAY 0xffffffff
inline void vTaskDelay(unsigned long ticks) { delay(ticks); }
inline void taskYIELD() {}

// Serial -> Webots console
class SimSerial {
 public:
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  void flush() { std::fflush(stdout); }
  int printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = std::vprintf(fmt, ap);
    va_end(ap);
    std::fflush(stdout);
    return n;
  }
  void print(const char *s) { std::printf("%s", s); }
  void print(char c) { std::printf("%c", c); }
  void print(int v) { std::printf("%d", v); }
  void print(unsigned int v) { std::printf("%u", v); }
  void print(long v) { std::printf("%ld", v); }
  void print(unsigned long v) { std::printf("%lu", v); }
  void print(long long v) { std::printf("%lld", v); }
  void print(unsigned long long v) { std::printf("%llu", v); }
  void print(double v, int digits = 2) { std::printf("%.*f", digits, v); }
  void print(bool v) { std::printf("%d", v ? 1 : 0); }
  void println() { std::printf("\n"); std::fflush(stdout); }
  template <class T> void println(T v) { print(v); println(); }
  void println(double v, int digits) { print(v, digits); println(); }
};
extern SimSerial Serial;
