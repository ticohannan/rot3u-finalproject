/*
  servo_bridge.ino  v0.3 - minimal servo bridge. Python (Jupyter) does the thinking.
  Robot: any ROT3U-type arm. Arduino Mega 2560 + PCA9685 @ 0x40.
  Channels: 0 base, 1 shoulder, 2 elbow, 3 wrist pitch, 4 wrist roll, 5 gripper.
  Copyright (c) 2026 Tico Hannan. BSD-3-Clause.

  STATUS: v0.1 ran on Sir Pinch-a-lot 2026-11-15 (Robot Demo 4 tracking). v0.2 (input shaping) and
  v0.3 (limits from the host) compiled for the ATmega2560 and run on a PC simulation with an emulated
  PCA9685; not yet run on an arm.

  v0.3: the sketch no longer holds one robot's pulse limits. Its built-in limits are the servo's
  absolute range, 500-2500 us on every channel, and the notebook sends the robot's own limits from its
  robot_config.h with LIM right after connecting (rot3u_demo4 robot.apply_limits). Opening the port
  resets the Mega and the limits with it: always connect through the notebook before moving anything.
  When typing commands in the Serial Monitor yourself, send your robot's LIM lines first.

  What it does: holds one target pulse per channel and moves each channel toward its target
  in the background, 50 times a second, with a speed limit and an acceleration limit (smooth
  start and stop). It knows nothing about angles, kinematics or cameras.
  Input shaping (v0.2, optional per channel): the arm's frame is springy and keeps swinging after a
  move. With SHAPE c f z, channel c's motion is sent as two half-size copies, the second one delayed by
  half a swing period, so the swing started by the first is cancelled by the second (a ZV shaper):
      out(t) = A1 * p(t) + A2 * p(t - Td/2)    K = exp(-z pi / sqrt(1 - z^2))
      A1 = 1 / (1 + K), A2 = K / (1 + K),      Td = 1 / (f sqrt(1 - z^2))
  p(t) is the speed- and acceleration-limited motion; f (Hz) and z (damping ratio) are measured with
  the camera (rot3u_demo4 measure_ringdown). Moves arrive about Td/2 later. A stop (X, Enter, unknown
  line) still holds at once: the delayed copy is dropped.

  Safety rules (same as servo_tester and ik_demo):
    - no pulse on a channel until it is started with ON (or adopted, already holding, at boot)
    - every target is clamped to the channel's limits; LIM sets them inside the built-in 500-2500 us
    - Enter alone, X, or any line it does not understand stops all motion; the arm HOLDS
    - nothing is ever released automatically; only OFF releases
    - opening the serial port resets the Mega; the PCA9685 keeps its outputs and the sketch
      adopts them, so a holding arm keeps holding

  Serial: 115200 baud, line ending Newline. Lines are at most 63 characters.
  Commands (pulses in microseconds):
    ON c us         start channel c at us (first pulse: the servo jumps there, support the arm)
    T a b c d e f   targets for channels 0..5; "-" leaves a channel alone; fewer than 6 is fine
    TC c us         target for one channel
    VA v a [c]      speed limit v (us/s) and acceleration limit a (us/s^2), all channels or c
    LIM c lo hi     narrow the limits of channel c; LIM alone prints all limits
    ST              print one state line (STATE also works)
    STREAM ms       print a state line every ms (20-5000); STREAM 0 stops
    SHAPE c f z     input shaping on channel c (or * for all): swing frequency f (0.5-10 Hz) and
                    damping z (0-0.7); SHAPE c 0 turns it off; SHAPE alone lists the settings
    X               stop: every target becomes the present position (Enter alone does the same)
    OFF c | OFF *   release channel c, or all channels (the arm goes limp: support it first)
    HELLO           print the HELLO line again
    ?               help
  Machine framing (same as ik_demo's host link): "@<id> <command>" is answered with "ACK,<id>"
  and then "DONE,<id>,OK" or "DONE,<id>,FAIL". Commands never block: motion runs in the background.
  State line: ST,<ms>,<now 0..5>,<target 0..5>,<active bits>,<moving 0|1>
*/

#include <Wire.h>
#include <math.h>

#define FW_NAME    "servo_bridge"
#define FW_VERSION "0.3"

// ---- built-in limits: the servo's absolute range. The robot's own limits come from the host (LIM).
const uint8_t  NUM_CH = 6;
const uint16_t HARD_MIN[NUM_CH] = { 500,  500,  500,  500,  500,  500};
const uint16_t HARD_MAX[NUM_CH] = {2500, 2500, 2500, 2500, 2500, 2500};
const float    V_DEFAULT = 300.0f;     // us/s    (about 27 deg/s)
const float    A_DEFAULT = 1200.0f;    // us/s^2

// ---- PCA9685
const uint8_t  PCA_ADDR = 0x40;
const uint8_t  REG_MODE1 = 0x00, REG_LED0 = 0x06, REG_PRE = 0xFE;
const float    OSC_HZ = 27000000.0f;
const uint8_t  PRESCALE_50HZ = 131;
const uint16_t STEP_MS = 20;           // motion update period
const uint8_t  HIST = 64;              // shaper history per channel: 64 x 20 ms = 1.28 s (f >= 0.5 Hz)

float    pos[NUM_CH];                  // pulse being written now (the profile's output, shaped if SHAPE is on)
float    raw[NUM_CH];                  // the speed- and acceleration-limited profile, before shaping
float    hist[NUM_CH][HIST];           // raw, one entry per update, newest at histAt
uint8_t  histAt = 0;
float    shA2[NUM_CH], shLag[NUM_CH];  // second-impulse weight (0 = off) and its delay in updates
float    shF[NUM_CH], shZ[NUM_CH];
float    vel[NUM_CH];                  // us/s
float    tgt[NUM_CH];                  // where the channel is going
float    vmax[NUM_CH], amax[NUM_CH];
uint16_t limLo[NUM_CH], limHi[NUM_CH];
uint16_t lastCounts[NUM_CH];
bool     active[NUM_CH];
uint8_t  psNow = PRESCALE_50HZ;
uint16_t streamMs = 0;
uint32_t lastStep = 0, lastStream = 0, lastByte = 0, lastI2cErr = 0;
char     buf[64];
uint8_t  blen = 0;
bool     overflow = false, tagged = false, lastWasCR = false;

// ------------------------------------------------------------ PCA9685
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

int readReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  Wire.requestFrom((int)PCA_ADDR, 1);
  return Wire.available() ? Wire.read() : -1;
}

void setCounts(uint8_t c, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(REG_LED0 + 4 * c);
  Wire.write(on & 0xFF); Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  if (Wire.endTransmission() != 0 && millis() - lastI2cErr > 1000) {
    lastI2cErr = millis();
    Serial.print(F("ERR,")); Serial.print(millis()); Serial.println(F(",I2C"));
  }
}

float usPerCount() { return 1000000.0f * (float)((uint16_t)psNow + 1) / OSC_HZ; }

void writeUs(uint8_t c, float us) {    // the only place a pulse is written
  if (us < 500) us = 500;              // absolute servo bounds; the channel limits act on the targets
  if (us > 2500) us = 2500;
  uint16_t n = (uint16_t)(us / usPerCount() + 0.5f);
  if (n != lastCounts[c]) { setCounts(c, 0, n); lastCounts[c] = n; }
}

void releaseCh(uint8_t c) { setCounts(c, 0, 4096); active[c] = false; vel[c] = 0; lastCounts[c] = 0; }
void holdAt(uint8_t c, float us);

// ------------------------------------------------------------- motion
float clampLim(uint8_t c, float us) {
  if (us < limLo[c]) us = limLo[c];
  if (us > limHi[c]) us = limHi[c];
  return us;
}

void holdAt(uint8_t c, float us) {    // profile, shaper history and output all at one pulse: no motion pending
  pos[c] = raw[c] = tgt[c] = us; vel[c] = 0;
  for (uint8_t i = 0; i < HIST; i++) hist[c][i] = us;
}

void stopAll() {                       // hold where the output is now; the shaper's delayed copy is dropped
  for (uint8_t c = 0; c < NUM_CH; c++) holdAt(c, pos[c]);
}

bool isMoving() {
  for (uint8_t c = 0; c < NUM_CH; c++) if (active[c] && (pos[c] != tgt[c] || raw[c] != tgt[c] || vel[c] != 0)) return true;
  return false;
}

float histBack(uint8_t c, float lag) { // raw profile 'lag' updates ago, linear between entries
  uint8_t n = (uint8_t)lag;
  float f = lag - n;
  float a = hist[c][(uint8_t)(histAt + HIST - n) % HIST];
  float b = hist[c][(uint8_t)(histAt + HIST - n - 1) % HIST];
  return a + (b - a) * f;
}

void setShape(uint8_t c, float f, float z) {
  if (f <= 0) { shA2[c] = 0; shF[c] = 0; shZ[c] = 0; return; }
  float wd = sqrt(1.0f - z * z);
  float K = exp(-z * PI / wd);
  shA2[c] = K / (1.0f + K);
  shLag[c] = (0.5f / (f * wd)) / ((float)STEP_MS / 1000.0f);
  if (shLag[c] > HIST - 2) shLag[c] = HIST - 2;
  shF[c] = f; shZ[c] = z;
}

// One update. The speed aims at the highest value from which the channel can still stop at the target
// with deceleration a, counted in whole updates:  v = sqrt(2 a d + (a dt / 2)^2) - a dt / 2,
// capped by vmax and by d / dt (never past the target), and it changes by at most a * dt per update.
void stepMotion(float dt) {
  histAt = (histAt + 1) % HIST;
  for (uint8_t c = 0; c < NUM_CH; c++) {
    if (!active[c]) { hist[c][histAt] = raw[c]; continue; }
    float err = tgt[c] - raw[c];
    float dvMax = amax[c] * dt;
    float d = fabs(err);
    float h = 0.5f * dvMax;
    float vdes = sqrt(2.0f * amax[c] * d + h * h) - h;
    if (vdes > vmax[c]) vdes = vmax[c];
    if (vdes > d / dt) vdes = d / dt;
    if (err < 0) vdes = -vdes;
    float dv = vdes - vel[c];
    if (dv > dvMax) dv = dvMax;
    if (dv < -dvMax) dv = -dvMax;
    vel[c] += dv;
    float np = raw[c] + vel[c] * dt;
    if (fabs(tgt[c] - np) < 0.02f && fabs(vel[c]) <= dvMax) { np = tgt[c]; vel[c] = 0; }   // arrived
    // never travel outward past a limit (an adopted channel may start outside: it only moves inward)
    if (vel[c] > 0 && np > limHi[c]) { np = (raw[c] > limHi[c]) ? raw[c] : (float)limHi[c]; vel[c] = 0; }
    if (vel[c] < 0 && np < limLo[c]) { np = (raw[c] < limLo[c]) ? raw[c] : (float)limLo[c]; vel[c] = 0; }
    raw[c] = np;
    hist[c][histAt] = np;
    if (shA2[c] > 0) {
      float out = (1.0f - shA2[c]) * np + shA2[c] * histBack(c, shLag[c]);
      pos[c] = (fabs(out - tgt[c]) < 0.02f && np == tgt[c] && vel[c] == 0) ? tgt[c] : out;
    } else pos[c] = np;
    writeUs(c, pos[c]);
  }
}

// ------------------------------------------------------------ replies
void printState() {
  uint8_t bits = 0;
  Serial.print(F("ST,")); Serial.print(millis());
  for (uint8_t c = 0; c < NUM_CH; c++) { Serial.print(','); Serial.print((long)(pos[c] + 0.5f)); if (active[c]) bits |= (1 << c); }
  for (uint8_t c = 0; c < NUM_CH; c++) { Serial.print(','); Serial.print((long)(tgt[c] + 0.5f)); }
  Serial.print(','); Serial.print(bits);
  Serial.print(','); Serial.println(isMoving() ? 1 : 0);
}

void printHello() {
  Serial.print(F("HELLO,")); Serial.print(F(FW_NAME)); Serial.print(','); Serial.print(F(FW_VERSION)); Serial.println(F(",2"));
}

void printHelp() {
  Serial.println(F("# ON c us | T a b c d e f (- = leave) | TC c us | VA v a [c] | LIM [c lo hi]"));
  Serial.println(F("# ST | STREAM ms | SHAPE c f z | X (or Enter) = stop and hold | OFF c | OFF * | HELLO | ?"));
}

void printLimits() {
  for (uint8_t c = 0; c < NUM_CH; c++) {
    Serial.print(F("# ch ")); Serial.print(c); Serial.print(F(": ")); Serial.print(limLo[c]); Serial.print(F(" - ")); Serial.print(limHi[c]);
    Serial.print(F(" us, v ")); Serial.print(vmax[c], 0); Serial.print(F(" us/s, a ")); Serial.print(amax[c], 0);
    Serial.print(active[c] ? F(" us/s^2, on") : F(" us/s^2, off"));
    if (shF[c] > 0) { Serial.print(F(", shaped ")); Serial.print(shF[c], 2); Serial.print(F(" Hz z ")); Serial.print(shZ[c], 2); }
    Serial.println();
  }
}

bool fail(const __FlashStringHelper *why) { Serial.print(F("# ")); Serial.println(why); return false; }
bool okay() { if (!tagged) Serial.println(F("# ok")); return true; }

// ----------------------------------------------------------- commands
bool isNum(const char *t) {            // digits with an optional sign and decimal point
  if (!t || !*t) return false;
  if (*t == '-' || *t == '+') t++;
  if (!*t) return false;
  for (; *t; t++) if (!isdigit(*t) && *t != '.') return false;
  return true;
}

int8_t nextCh() {                      // next token as a channel number, -1 if it is not one
  char *t = strtok(NULL, " ,");
  if (!t || !isdigit(t[0]) || t[1] != 0 || t[0] - '0' >= NUM_CH) return -1;
  return t[0] - '0';
}

bool cmdTargets() {
  float v[NUM_CH]; bool has[NUM_CH]; uint8_t n = 0;
  for (uint8_t c = 0; c < NUM_CH; c++) has[c] = false;
  for (uint8_t c = 0; c < NUM_CH; c++) {
    char *t = strtok(NULL, " ,");
    if (!t) break;
    n++;
    if (t[0] == '-' && t[1] == 0) continue;
    if (!isNum(t)) return fail(F("T a b c d e f : numbers in us, or - to leave a channel"));
    v[c] = atof(t); has[c] = true;
  }
  if (!n) return fail(F("T a b c d e f : numbers in us, or - to leave a channel"));
  for (uint8_t c = 0; c < NUM_CH; c++)
    if (has[c] && !active[c]) return fail(F("a channel in that list is not started - ON c us first; nothing changed"));
  for (uint8_t c = 0; c < NUM_CH; c++) if (has[c]) tgt[c] = clampLim(c, v[c]);
  return okay();
}

bool execute(char *s) {
  char *cmd = strtok(s, " ,");
  if (!cmd) { stopAll(); if (!tagged) Serial.println(F("# stopped - holding")); return true; }
  for (char *p = cmd; *p; p++) *p = toupper(*p);

  if (!strcmp(cmd, "T")) return cmdTargets();
  if (!strcmp(cmd, "ST") || !strcmp(cmd, "STATE")) { printState(); return true; }
  if (!strcmp(cmd, "X")) { stopAll(); if (!tagged) Serial.println(F("# stopped - holding")); return true; }
  if (!strcmp(cmd, "TC")) {
    int8_t c = nextCh(); char *t = strtok(NULL, " ,");
    if (c < 0 || !isNum(t)) return fail(F("TC c us"));
    if (!active[c]) return fail(F("that channel is not started - ON c us first"));
    tgt[c] = clampLim(c, atof(t));
    return okay();
  }
  if (!strcmp(cmd, "ON")) {
    int8_t c = nextCh(); char *t = strtok(NULL, " ,");
    if (c < 0 || !isNum(t)) return fail(F("ON c us"));
    if (active[c]) return fail(F("that channel is already on - use TC or T to move it"));
    holdAt(c, clampLim(c, atof(t))); active[c] = true;
    writeUs(c, pos[c]);
    Serial.print(F("# ch ")); Serial.print(c); Serial.print(F(" on at ")); Serial.println((long)(pos[c] + 0.5f));
    return true;
  }
  if (!strcmp(cmd, "OFF")) {
    char *t = strtok(NULL, " ,");
    if (t && t[0] == '*') { for (uint8_t c = 0; c < NUM_CH; c++) releaseCh(c); Serial.println(F("# all channels released")); return true; }
    if (!t || !isdigit(t[0]) || t[1] != 0 || t[0] - '0' >= NUM_CH) return fail(F("OFF c | OFF *"));
    releaseCh(t[0] - '0');
    return okay();
  }
  if (!strcmp(cmd, "VA")) {
    char *tv = strtok(NULL, " ,"), *ta = strtok(NULL, " ,"), *tc = strtok(NULL, " ,");
    if (!isNum(tv) || !isNum(ta)) return fail(F("VA v a [c]   v 10-3000 us/s, a 50-30000 us/s^2"));
    float v = atof(tv), a = atof(ta);
    if (v < 10 || v > 3000 || a < 50 || a > 30000) return fail(F("VA v a [c]   v 10-3000 us/s, a 50-30000 us/s^2"));
    if (tc && (!isdigit(tc[0]) || tc[1] != 0 || tc[0] - '0' >= NUM_CH)) return fail(F("VA v a [c]   c is 0-5"));
    for (uint8_t c = 0; c < NUM_CH; c++) if (!tc || c == tc[0] - '0') { vmax[c] = v; amax[c] = a; }
    return okay();
  }
  if (!strcmp(cmd, "LIM")) {
    char *tc = strtok(NULL, " ,");
    if (!tc) { printLimits(); return true; }
    char *tl = strtok(NULL, " ,"), *th = strtok(NULL, " ,");
    if (!isdigit(tc[0]) || tc[1] != 0 || tc[0] - '0' >= NUM_CH || !isNum(tl) || !isNum(th)) return fail(F("LIM c lo hi"));
    uint8_t c = tc[0] - '0'; long lo = atol(tl), hi = atol(th);
    if (lo < HARD_MIN[c] || hi > HARD_MAX[c] || lo >= hi) return fail(F("LIM c lo hi : must be inside the built-in limits, lo < hi"));
    limLo[c] = lo; limHi[c] = hi;
    if (active[c]) tgt[c] = clampLim(c, tgt[c]);
    return okay();
  }
  if (!strcmp(cmd, "STREAM")) {
    char *t = strtok(NULL, " ,");
    long ms = isNum(t) ? atol(t) : -1;
    if (ms != 0 && (ms < 20 || ms > 5000)) return fail(F("STREAM ms : 20-5000, or 0 to stop"));
    streamMs = (uint16_t)ms; lastStream = millis();
    return okay();
  }
  if (!strcmp(cmd, "SHAPE")) {
    char *tc = strtok(NULL, " ,"), *tf = strtok(NULL, " ,"), *tz = strtok(NULL, " ,");
    if (!tc) { printLimits(); return true; }
    bool all = (tc[0] == '*' && tc[1] == 0);
    if (!all && (!isdigit(tc[0]) || tc[1] != 0 || tc[0] - '0' >= NUM_CH)) return fail(F("SHAPE c f z   c 0-5 or *"));
    if (!isNum(tf)) return fail(F("SHAPE c f z   f 0.5-10 Hz (0 = off), z 0-0.7"));
    float f = atof(tf), z = isNum(tz) ? atof(tz) : 0.0f;
    if (f != 0 && (f < 0.5f || f > 10.0f || z < 0 || z > 0.7f)) return fail(F("SHAPE c f z   f 0.5-10 Hz (0 = off), z 0-0.7"));
    for (uint8_t c = 0; c < NUM_CH; c++)
      if (all || c == tc[0] - '0') { if (active[c]) holdAt(c, pos[c]); setShape(c, f, z); }
    return okay();
  }
  if (!strcmp(cmd, "HELLO")) { printHello(); return true; }
  if (!strcmp(cmd, "?") || !strcmp(cmd, "HELP")) { printHelp(); return true; }

  stopAll();                           // anything not understood stops the arm
  Serial.print(F("# unknown command - stopped, holding: ")); Serial.println(cmd);
  return false;
}

void handleLine(char *line) {
  char *p = line;
  uint16_t id = 0;
  while (*p == ' ') p++;
  tagged = (*p == '@');
  if (tagged) {
    id = (uint16_t)strtoul(p + 1, &p, 10);
    Serial.print(F("ACK,")); Serial.println(id);
  }
  bool ok = execute(p);
  if (tagged) { Serial.print(F("DONE,")); Serial.print(id); Serial.println(ok ? F(",OK") : F(",FAIL")); }
  tagged = false;
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    lastByte = millis();
    if (c == '\n' || c == '\r') {
      bool lfOfCrLf = (c == '\n' && lastWasCR);     // "Both NL & CR": the pair is one line end
      lastWasCR = (c == '\r');
      if (lfOfCrLf) continue;
      if (overflow) {
        stopAll(); overflow = false; blen = 0;
        Serial.print(F("ERR,")); Serial.print(millis()); Serial.println(F(",LINE_TOO_LONG"));
      } else { buf[blen] = 0; blen = 0; handleLine(buf); }   // an empty line (Enter alone) = stop
      continue;
    }
    lastWasCR = false;
    if (blen < sizeof(buf) - 1) buf[blen++] = c; else overflow = true;
  }
  // a stray key with no line ending (Serial Monitor set to "No line ending") still counts after 60 ms
  if (blen && millis() - lastByte > 60) { buf[blen] = 0; blen = 0; overflow = false; handleLine(buf); }
}

// --------------------------------------------------------- setup/loop
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(200);
  Serial.print(F("# READY ")); Serial.print(F(FW_NAME)); Serial.print(F(" v")); Serial.println(F(FW_VERSION));
  for (uint8_t c = 0; c < NUM_CH; c++) {
    limLo[c] = HARD_MIN[c]; limHi[c] = HARD_MAX[c];
    vmax[c] = V_DEFAULT; amax[c] = A_DEFAULT;
    holdAt(c, 1500); active[c] = false; lastCounts[c] = 0; setShape(c, 0, 0);
  }
  int pre = readReg(REG_PRE), mode = readReg(REG_MODE1);
  if (pre > 120 && pre < 145 && mode >= 0 && !(mode & 0x10)) {   // already running (awake, 50 Hz): adopt, do not reset
    psNow = (uint8_t)pre;
    if (!(mode & 0x20)) writeReg(REG_MODE1, (mode & 0x7F) | 0x20);   // multi-byte reads and writes need auto-increment
    for (uint8_t c = 0; c < NUM_CH; c++) {
      Wire.beginTransmission(PCA_ADDR); Wire.write(REG_LED0 + 4 * c);
      if (Wire.endTransmission() != 0) continue;
      Wire.requestFrom((int)PCA_ADDR, 4);
      if (Wire.available() < 4) continue;
      uint8_t onL = Wire.read(), onH = Wire.read(), offL = Wire.read(), offH = Wire.read();
      uint16_t on  = ((uint16_t)(onH  & 0x0F) << 8) | onL;
      uint16_t off = ((uint16_t)(offH & 0x0F) << 8) | offL;
      if ((onH & 0x10) || (offH & 0x10) || on != 0 || off == 0) continue;
      float us = (float)off * usPerCount();
      if (us < 495 || us > 2505) continue;
      holdAt(c, us); active[c] = true; lastCounts[c] = off;
      Serial.print(F("# adopted ch ")); Serial.print(c); Serial.print(F(" at ")); Serial.println((long)(us + 0.5f));
    }
    Serial.println(F("# PCA9685 was running: live channels adopted, nothing moved"));
  } else {                                          // cold board: configure, outputs off
    writeReg(REG_MODE1, 0x10);
    writeReg(REG_PRE, PRESCALE_50HZ);
    writeReg(REG_MODE1, 0x00);
    delay(5);
    writeReg(REG_MODE1, 0xA0);
    psNow = PRESCALE_50HZ;
    for (uint8_t c = 0; c < NUM_CH; c++) releaseCh(c);
    Serial.println(F("# PCA9685 configured, all outputs off - start channels with ON c us"));
  }
  printHelp();
  lastStep = millis();
  printHello();
}

void loop() {
  readSerial();
  uint32_t now = millis();
  if (now - lastStep >= STEP_MS) {
    float dt = (float)(now - lastStep) / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    lastStep = now;
    stepMotion(dt);
  }
  if (streamMs && now - lastStream >= streamMs) { lastStream = now; printState(); }
}
