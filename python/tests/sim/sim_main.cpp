// PC simulation of servo_bridge.ino for tests: the real sketch, an emulated PCA9685, simulated time.
// Usage: sim_servo_bridge [state file]
//   stdin lines:  "@adv N"  run loop() for N ms of simulated time
//                 (every stdin line is answered with "@t <ms>" once it has been handled)
//                 "@raw xyz" send the characters with no line ending; "@crlf xyz" end the line with CR LF
//                 anything else is sent to the sketch as a serial line
//   stdout: the sketch's serial output, plus "W,<ch>,<off counts>,<ms>" for every PCA9685 write.
// The state file (PCA9685 registers) is loaded at start and saved at exit, so two runs in a row
// behave like an Arduino reset with the servo board still powered.
// Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
#include "Arduino.h"
#include "Wire.h"
#include <iostream>
std::deque<char> g_in; SerialShim Serial; WireShim Wire;
static unsigned long g_ms = 0;
void g_char(char c) { if (c != '\r') std::cout << c; }
unsigned long millis() { return g_ms; }
void delay(unsigned long ms) { g_ms += ms; }
void yield() {}
uint8_t WireShim::endTransmission(bool) {
  if (addr != devAddr) return 2;
  if (tx.empty()) return 0;
  ptr = tx[0];
  for (size_t i = 1; i < tx.size(); i++) {
    uint8_t r = (uint8_t)(ptr + i - 1);
    if (r == 0xFE && !(reg[0] & 0x10)) continue;      // prescale only writable while asleep
    reg[r] = tx[i];
  }
  if (tx.size() >= 2 && tx[0] == 0x00) reg[0] &= 0x7F;
  if (tx.size() == 5 && ptr >= 6 && ptr < 6 + 4 * 16) {
    int ch = (ptr - 6) / 4; int off = tx[3] | (tx[4] << 8);
    std::cout << "W," << ch << "," << off << "," << g_ms << "\n";
  }
  return 0;
}
#include "sketch_host.cpp"
int main(int argc, char **argv) {
  if (argc > 1) { FILE *s = fopen(argv[1], "rb"); if (s) { if (fread(Wire.reg, 1, 256, s) != 256) {} fclose(s); } }
  setup();
  std::cout << "@t " << g_ms << std::endl;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.rfind("@adv ", 0) == 0) {
      unsigned long end = g_ms + atol(line.c_str() + 5);
      while (g_ms < end) { loop(); g_ms += 1; }
      std::cout << "@t " << g_ms << std::endl;
      continue;
    }
    if (line.rfind("@raw ", 0) == 0) { for (char c : line.substr(5)) g_in.push_back(c); std::cout << "@t " << g_ms << std::endl; continue; }
    if (line.rfind("@crlf ", 0) == 0) {               // a line ended by CR LF, as "Both NL & CR" sends it
      for (char c : line.substr(6)) g_in.push_back(c);
      g_in.push_back('\r'); g_in.push_back('\n'); loop();
      std::cout << "@t " << g_ms << std::endl; continue;
    }
    for (char c : line) g_in.push_back(c);
    g_in.push_back('\n');
    loop();                                           // the sketch reads the line now; time does not move
    std::cout << "@t " << g_ms << std::endl;
  }
  if (argc > 1) { FILE *s = fopen(argv[1], "wb"); if (s) { fwrite(Wire.reg, 1, 256, s); fclose(s); } }
}
