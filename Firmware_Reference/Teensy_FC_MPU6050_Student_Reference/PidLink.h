/*
  PidLink.h - PID Tuner Protocol v1 for Arduino / Teensy
  Single header, no external libraries, no dynamic allocation.

  Drop this next to your sketch, describe your firmware once, register a few
  callbacks, and the browser tuner will build its whole interface around your
  board. Your pins, sensors, libraries, architecture and control law are
  entirely your own business.

    #include "PidLink.h"

    static const char DESCRIPTOR[] = "{ \"protocol\":1, ... }";
    PidLink link;

    void setup() {
      Serial.begin(115200);
      link.begin(DESCRIPTOR);
      link.onPidSet(myPidSet);
      link.onPidGet(myPidGet);
      link.onImuSample(myImuSample);   // only if you want calibration
      link.onIsStopped(myIsStopped);   // required before calibration runs
    }

    void loop() {
      link.poll();                     // cheap, non-blocking, call often
      ...your control loop...
      link.telemetryBegin();
      link.telemetryValue(micros());
      link.telemetryValue(angle, 3);
      link.telemetryEnd();
    }

  See PROTOCOL.md for the wire format.
*/

#ifndef PIDLINK_H
#define PIDLINK_H

#include <Arduino.h>

// Set to 0 for boards without EEPROM, or if you do not want persistence.
#ifndef PIDLINK_USE_EEPROM
#define PIDLINK_USE_EEPROM 1
#endif

#if PIDLINK_USE_EEPROM
#include <EEPROM.h>
#endif

// Where the calibration block lives in EEPROM. Move it if you store your own
// data there.
#ifndef PIDLINK_EEPROM_ADDR
#define PIDLINK_EEPROM_ADDR 0
#endif

// Bytes of descriptor per $D line. 200 suits USB. Drop it if you ever pipe the
// descriptor through a constrained link.
#ifndef PIDLINK_CHUNK
#define PIDLINK_CHUNK 200
#endif

#ifndef PIDLINK_LINE_MAX
#define PIDLINK_LINE_MAX 160
#endif

#ifndef PIDLINK_STREAM
#define PIDLINK_STREAM Serial
#endif

// The flight sketches supply a bounded output queue. Keep standalone users
// compatible, while allowing those sketches to reject work before mutation.
#ifndef PIDLINK_TX_READY
#define PIDLINK_TX_READY(bytes) true
#endif

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/* One raw IMU reading, in physical units, before calibration offsets.
   Fill in whatever your sensor gives you; leave the rest at zero. */
struct PidLinkImuSample {
  float gyro[3]  = {0, 0, 0};   // deg/s, x y z
  float accel[3] = {0, 0, 0};   // g, x y z
  float roll = 0, pitch = 0;    // deg, your fused attitude if you have one
  bool hasAttitude = false;
};

/* Calibration offsets. Subtract these from raw readings. */
struct PidLinkCalibration {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint16_t flags = 0;
  float gyro[3]  = {0, 0, 0};
  float accel[3] = {0, 0, 0};
  float roll = 0, pitch = 0;
  uint32_t crc = 0;
};

/* Parsed name=value pairs from a PID,SET. Positional order is never assumed,
   so adding a parameter later cannot break an older tuner. */
struct PidLinkParams {
  static const uint8_t MAX = 10;
  static const uint8_t NAME_MAX = 14;
  char name[MAX][NAME_MAX];
  float value[MAX];
  bool valid[MAX];
  uint8_t count = 0;

  bool get(const char *n, float &out) const {
    for (uint8_t i = 0; i < count; i++) {
      if (valid[i] && strcmp(name[i], n) == 0) { out = value[i]; return true; }
    }
    return false;
  }
  bool has(const char *n) const { float t; return get(n, t); }
  /* True if any pair failed to parse as a number. Reject the whole SET when
     this is set: String-style conversions turn a typo into 0.0, which would
     silently mean "set this gain to zero". */
  bool anyMalformed() const {
    for (uint8_t i = 0; i < count; i++) if (!valid[i]) return true;
    return false;
  }
};

enum PidLinkStatus : uint8_t {
  PIDLINK_APPLIED = 0,
  PIDLINK_CURRENT = 1,
  PIDLINK_RESET = 2,
  PIDLINK_REJECTED = 4,
  PIDLINK_UNKNOWN_CONTROLLER = 5
};

// ---------------------------------------------------------------------------

class PidLink {
public:
  typedef void (*PidHandler)(const char *controller, const PidLinkParams &params);
  typedef void (*SimpleHandler)(const char *controller);
  typedef bool (*ImuSampleHandler)(PidLinkImuSample &out);
  typedef bool (*BoolHandler)();
  typedef void (*VoidHandler)();
  typedef void (*FloatHandler)(float value);
  typedef bool (*RateHandler)(uint32_t hz);

  void begin(const char *descriptorJson) {
    _descriptor = descriptorJson;
    _descriptorLen = descriptorJson ? strlen(descriptorJson) : 0;
    _hash = fnv1a(descriptorJson, _descriptorLen);
    extractString("\"name\"", _name, sizeof(_name), "unnamed");
    extractString("\"version\"", _version, sizeof(_version), "0");
    loadCalibration();
  }

  // -- callbacks ------------------------------------------------------------
  void onPidSet(PidHandler h)        { _pidSet = h; }
  void onPidGet(SimpleHandler h)     { _pidGet = h; }
  void onPidReset(SimpleHandler h)   { _pidReset = h; }
  void onStop(VoidHandler h)         { _stop = h; }
  void onResume(VoidHandler h)       { _resume = h; }
  void onSetTarget(FloatHandler h)   { _setTarget = h; }
  void onZeroSetpoint(VoidHandler h) { _zeroSetpoint = h; }
  void onTelemetryRate(RateHandler h){ _telemRate = h; }
  /* Must return a fresh raw IMU reading. Required for calibration. */
  void onImuSample(ImuSampleHandler h) { _imuSample = h; }
  /* Must return true when outputs are idle. Calibration refuses to run
     otherwise, because calibrating a live rig is never correct. */
  void onIsStopped(BoolHandler h)    { _isStopped = h; }

  // -- main loop ------------------------------------------------------------
  /* Non-blocking. Consumes only bytes already buffered, and advances a
     calibration if one is running. Safe to call from a fast control loop. */
  void poll() {
    uint16_t budget = 64;
    while (budget-- > 0 && PIDLINK_STREAM.available() > 0) {
      const char c = (char)PIDLINK_STREAM.read();
      if (_discardLine) {
        if (c == '\n' || c == '\r') _discardLine = false;
        continue;
      }
      if (c == '\n' || c == '\r') {
        if (_len > 0) { _line[_len] = '\0'; handleLine(_line); _len = 0; }
      } else if (_len < PIDLINK_LINE_MAX - 1) {
        _line[_len++] = c;
      } else {
        _len = 0;
        _discardLine = true;
        error("LINE_TOO_LONG");
      }
    }
    if (_calState != CAL_IDLE) serviceCalibration();
    serviceDescriptor();
  }

  // -- telemetry ------------------------------------------------------------
  void telemetryBegin() { PIDLINK_STREAM.print(F("$T")); }
  void telemetryValue(float v, uint8_t decimals = 3) {
    PIDLINK_STREAM.print(',');
    if (isfinite(v)) PIDLINK_STREAM.print(v, decimals);
  }
  void telemetryValue(long v)     { PIDLINK_STREAM.print(','); PIDLINK_STREAM.print(v); }
  void telemetryValue(int v)      { PIDLINK_STREAM.print(','); PIDLINK_STREAM.print(v); }
  void telemetryValue(uint32_t v) { PIDLINK_STREAM.print(','); PIDLINK_STREAM.print(v); }
  void telemetryValue(bool v)     { PIDLINK_STREAM.print(','); PIDLINK_STREAM.print(v ? 1 : 0); }
  void telemetryEnd()             { PIDLINK_STREAM.println(); }

  // -- replies --------------------------------------------------------------
  void pidReplyBegin(const char *controller, uint8_t code) {
    PIDLINK_STREAM.print(F("$P,"));
    PIDLINK_STREAM.print(controller);
    PIDLINK_STREAM.print(',');
    PIDLINK_STREAM.print(code);
  }
  void pidReplyParam(const char *name, float value, uint8_t decimals = 6) {
    PIDLINK_STREAM.print(',');
    PIDLINK_STREAM.print(name);
    PIDLINK_STREAM.print('=');
    PIDLINK_STREAM.print(value, decimals);
  }
  void pidReplyEnd() { PIDLINK_STREAM.println(); }

  void state(bool stopped, bool canResume, const char *reason = "") {
    PIDLINK_STREAM.print(F("$S,STATE,stopped="));
    PIDLINK_STREAM.print(stopped ? 1 : 0);
    PIDLINK_STREAM.print(F(",can_resume="));
    PIDLINK_STREAM.print(canResume ? 1 : 0);
    PIDLINK_STREAM.print(F(",reason="));
    PIDLINK_STREAM.println(reason);
  }
  void error(const char *reason) {
    PIDLINK_STREAM.print(F("$E,"));
    PIDLINK_STREAM.println(reason);
  }
  void ack(const char *what) {
    PIDLINK_STREAM.print(F("$K,"));
    PIDLINK_STREAM.println(what);
  }

  // -- calibration ----------------------------------------------------------
  const PidLinkCalibration &calibration() const { return _cal; }
  bool calibrationRunning() const { return _calState != CAL_IDLE; }
  bool calibrationValid() const { return _cal.magic == CAL_MAGIC; }

  /* Correct a raw sample in place. Call this on every reading, so the rest of
     your control code only ever sees calibrated data. */
  void applyCalibration(PidLinkImuSample &s) const {
    if (!calibrationValid()) return;
    for (uint8_t i = 0; i < 3; i++) {
      s.gyro[i] -= _cal.gyro[i];
      s.accel[i] -= _cal.accel[i];
    }
    s.roll -= _cal.roll;
    s.pitch -= _cal.pitch;
  }
  /* Convenience for firmware that only tracks one axis. */
  float correctedRoll(float rawRoll) const {
    return calibrationValid() ? rawRoll - _cal.roll : rawRoll;
  }
  float correctedPitch(float rawPitch) const {
    return calibrationValid() ? rawPitch - _cal.pitch : rawPitch;
  }
  float correctedGyro(uint8_t axis, float raw) const {
    return (calibrationValid() && axis < 3) ? raw - _cal.gyro[axis] : raw;
  }

  /* Run a calibration to completion, for use from setup(). Everything else is
     non-blocking, but at boot there is no control loop to starve and a sketch
     with no stored offsets wants them before it starts flying. Returns false if
     the rig was moving, the sensor failed, or no offsets resulted. */
  bool calibrateNow(uint32_t timeoutMs = 6000) {
    startCalibration();
    const uint32_t t0 = millis();
    while (_calState != CAL_IDLE && (millis() - t0) < timeoutMs) {
      serviceCalibration();
    }
    if (_calState != CAL_IDLE) { _calState = CAL_IDLE; calFail("TIMEOUT"); return false; }
    return calibrationValid();
  }

  void clearCalibration() {
    _cal = PidLinkCalibration();
    _calAtMs = 0;
#if PIDLINK_USE_EEPROM
    PidLinkCalibration blank;
    EEPROM.put(PIDLINK_EEPROM_ADDR, blank);
#endif
    reportCalStatus();
  }

  bool saveCalibration() {
    if (_calState != CAL_IDLE || (_isStopped && !_isStopped())) return false;
#if PIDLINK_USE_EEPROM
    if (!calibrationValid()) return false;
    _cal.crc = calCrc(_cal);
    EEPROM.put(PIDLINK_EEPROM_ADDR, _cal);
    _cal.flags |= 1;
    reportCalStatus();
    return true;
#else
    return false;
#endif
  }

  // Tunables for the stillness check.
  float calMaxGyroDps = 2.0f;      // reject if any axis exceeds this
  uint16_t calSamples = 400;       // samples to average
  uint16_t calSampleIntervalUs = 2500;

private:
  static const uint32_t CAL_MAGIC = 0x504C4331UL;  // "PLC1"
  enum CalState : uint8_t { CAL_IDLE, CAL_RUNNING };

  const char *_descriptor = nullptr;
  size_t _descriptorLen = 0;
  uint32_t _hash = 0;
  char _name[32] = "unnamed";
  char _version[16] = "0";

  char _line[PIDLINK_LINE_MAX];
  uint16_t _len = 0;
  bool _discardLine = false;

  PidHandler _pidSet = nullptr;
  SimpleHandler _pidGet = nullptr, _pidReset = nullptr;
  VoidHandler _stop = nullptr, _resume = nullptr, _zeroSetpoint = nullptr;
  FloatHandler _setTarget = nullptr;
  RateHandler _telemRate = nullptr;
  ImuSampleHandler _imuSample = nullptr;
  BoolHandler _isStopped = nullptr;

  PidLinkCalibration _cal;
  uint32_t _calAtMs = 0;

  CalState _calState = CAL_IDLE;
  uint16_t _calCount = 0;
  uint32_t _calNextUs = 0;
  uint8_t _calLastPct = 255;
  double _accGyro[3] = {0, 0, 0};
  double _accAccel[3] = {0, 0, 0};
  double _accRoll = 0, _accPitch = 0;
  bool _calSawAttitude = false;

  // -- hashing / parsing helpers -------------------------------------------
  static uint32_t fnv1a(const char *s, size_t n) {
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < n; i++) { h ^= (uint8_t)s[i]; h *= 16777619UL; }
    return h;
  }

  /* Pull a short string value straight out of the descriptor text. Avoids
     pulling in a JSON parser just to learn our own name. */
  void extractString(const char *key, char *out, size_t outLen, const char *fallback) {
    strncpy(out, fallback, outLen - 1);
    out[outLen - 1] = '\0';
    if (!_descriptor) return;
    const char *k = strstr(_descriptor, key);
    if (!k) return;
    const char *colon = strchr(k, ':');
    if (!colon) return;
    const char *q1 = strchr(colon, '"');
    if (!q1) return;
    const char *q2 = strchr(q1 + 1, '"');
    if (!q2) return;
    size_t n = (size_t)(q2 - q1 - 1);
    if (n >= outLen) n = outLen - 1;
    memcpy(out, q1 + 1, n);
    out[n] = '\0';
  }

  static bool parseNumber(const char *text, float &out) {
    if (!text || !*text) return false;
    char *end = nullptr;
    out = strtof(text, &end);
    if (end == text) return false;
    while (*end && isspace((unsigned char)*end)) ++end;
    return *end == '\0' && isfinite(out);
  }

  // -- command dispatch -----------------------------------------------------
  void handleLine(char *line) {
    while (*line == ' ') line++;
    if (!*line) return;

    char *fields[PidLinkParams::MAX + 3] = {};
    uint8_t n = 0;
    fields[n++] = line;
    for (char *p = line; *p && n < PidLinkParams::MAX + 3; ++p) {
      if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
    }

    // STOP is never held behind host USB backpressure. Other requests must
    // reserve reply capacity BEFORE modifying gains, calibration, or outputs.
    if (equalsCI(fields[0], "OUT") && n >= 2 && equalsCI(fields[1], "STOP")) {
      if (_stop) _stop(); else error("NOT_SUPPORTED");
      return;
    }
    if (!PIDLINK_TX_READY(6144)) { error("USB_BUSY_RETRY"); return; }

    if (equalsCI(fields[0], "SYS")) {
      if (n >= 2 && equalsCI(fields[1], "DESCRIBE")) { sendDescriptor(); return; }
      if (n >= 2 && equalsCI(fields[1], "ID"))       { sendIdentity(); return; }
      if (n >= 2 && equalsCI(fields[1], "PING"))     { ack("PONG"); return; }
      error("BAD_SYS");
      return;
    }

    if (equalsCI(fields[0], "PID")) {
      if (n < 3) { error("BAD_PID"); return; }
      if (equalsCI(fields[1], "GET")) {
        if (_pidGet) _pidGet(fields[2]); else error("NOT_SUPPORTED");
        return;
      }
      if (equalsCI(fields[1], "RESET")) {
        if (_pidReset) _pidReset(fields[2]); else error("NOT_SUPPORTED");
        return;
      }
      if (equalsCI(fields[1], "SET")) {
        if (!_pidSet) { error("NOT_SUPPORTED"); return; }
        PidLinkParams p;
        for (uint8_t i = 3; i < n && p.count < PidLinkParams::MAX; i++) {
          char *eq = strchr(fields[i], '=');
          if (!eq) continue;
          *eq = '\0';
          strncpy(p.name[p.count], fields[i], PidLinkParams::NAME_MAX - 1);
          p.name[p.count][PidLinkParams::NAME_MAX - 1] = '\0';
          p.valid[p.count] = parseNumber(eq + 1, p.value[p.count]);
          p.count++;
        }
        _pidSet(fields[2], p);
        return;
      }
      error("BAD_PID");
      return;
    }

    if (equalsCI(fields[0], "CAL")) {
      if (n >= 2 && equalsCI(fields[1], "START"))  { startCalibration(); return; }
      if (n >= 2 && equalsCI(fields[1], "STATUS")) { reportCalStatus(); return; }
      if (n >= 2 && equalsCI(fields[1], "CLEAR")) {
        if (_isStopped && !_isStopped()) { calFail("NOT_STOPPED"); return; }
        clearCalibration();
        return;
      }
      if (n >= 2 && equalsCI(fields[1], "SAVE")) {
        if (!saveCalibration()) error("SAVE_FAILED");
        return;
      }
      error("BAD_CAL");
      return;
    }

    if (equalsCI(fields[0], "OUT")) {
      if (n >= 2 && equalsCI(fields[1], "STOP"))   { if (_stop) _stop(); else error("NOT_SUPPORTED"); return; }
      if (n >= 2 && equalsCI(fields[1], "RESUME")) { if (_resume) _resume(); else error("NOT_SUPPORTED"); return; }
      error("BAD_OUT");
      return;
    }

    if (equalsCI(fields[0], "SET")) {
      if (n >= 3 && equalsCI(fields[1], "TARGET")) {
        float v;
        if (!parseNumber(fields[2], v)) { error("BAD_TARGET"); return; }
        if (_setTarget) _setTarget(v); else error("NOT_SUPPORTED");
        return;
      }
      if (n >= 2 && equalsCI(fields[1], "ZERO")) {
        if (_zeroSetpoint) _zeroSetpoint(); else error("NOT_SUPPORTED");
        return;
      }
      error("BAD_SET");
      return;
    }

    if (equalsCI(fields[0], "TELEM") && n >= 3 && equalsCI(fields[1], "RATE")) {
      float hz;
      if (!parseNumber(fields[2], hz)) { error("BAD_RATE"); return; }
      if (!_telemRate) { error("NOT_SUPPORTED"); return; }
      if (hz < 1.0f || hz > 2000.0f || floorf(hz) != hz) { error("RATE_REJECTED"); return; }
      if (!_telemRate((uint32_t)hz)) { error("RATE_REJECTED"); return; }
      PIDLINK_STREAM.print(F("$K,TELEM,RATE,"));
      PIDLINK_STREAM.println((uint32_t)hz);
      return;
    }

    error("UNKNOWN_COMMAND");
  }

  static bool equalsCI(const char *a, const char *b) {
    if (!a || !b) return false;
    while (*a && *b) {
      if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
      a++; b++;
    }
    return *a == *b;
  }

  // -- discovery ------------------------------------------------------------
  void sendDescriptor() {
    if (!_descriptor || _descriptorLen == 0) { error("NO_DESCRIPTOR"); return; }
    if (_descriptorSending) { error("DESCRIPTOR_BUSY"); return; }
    _descriptorNextChunk = 0;
    _descriptorSending = true;
  }

  void serviceDescriptor() {
    if (!_descriptorSending || !PIDLINK_TX_READY(PIDLINK_CHUNK + 32)) return;
    const uint16_t total = (uint16_t)((_descriptorLen + PIDLINK_CHUNK - 1) / PIDLINK_CHUNK);
    // One bounded chunk per foreground pass; never serialize the whole
    // multi-kilobyte descriptor in one flight-control iteration.
    const uint16_t i = _descriptorNextChunk;
    {
      const size_t off = (size_t)i * PIDLINK_CHUNK;
      size_t len = _descriptorLen - off;
      if (len > PIDLINK_CHUNK) len = PIDLINK_CHUNK;
      PIDLINK_STREAM.print(F("$D,"));
      PIDLINK_STREAM.print(i);
      PIDLINK_STREAM.print(',');
      PIDLINK_STREAM.print(total);
      PIDLINK_STREAM.print(',');
      PIDLINK_STREAM.write((const uint8_t *)(_descriptor + off), len);
      PIDLINK_STREAM.println();
    }
    if (++_descriptorNextChunk >= total) _descriptorSending = false;
  }

  uint16_t _descriptorNextChunk = 0;
  bool _descriptorSending = false;

  void sendIdentity() {
    PIDLINK_STREAM.print(F("$I,"));
    PIDLINK_STREAM.print(_name);
    PIDLINK_STREAM.print(',');
    PIDLINK_STREAM.print(_version);
    PIDLINK_STREAM.print(',');
    char hex[9];
    snprintf(hex, sizeof(hex), "%08lx", (unsigned long)_hash);
    PIDLINK_STREAM.println(hex);
  }

public:
  uint32_t descriptorHash() const { return _hash; }
  const char *firmwareName() const { return _name; }
  const char *firmwareVersion() const { return _version; }

private:
  // -- calibration ----------------------------------------------------------
  void startCalibration() {
    if (!_imuSample) { calFail("NOT_SUPPORTED"); return; }
    // Calibrating a rig whose actuators are live is never correct, and the
    // vibration alone would poison the average.
    if (_isStopped && !_isStopped()) { calFail("NOT_STOPPED"); return; }
    _calState = CAL_RUNNING;
    _calCount = 0;
    _calLastPct = 255;
    _calNextUs = micros();
    _calSawAttitude = false;
    for (uint8_t i = 0; i < 3; i++) { _accGyro[i] = 0; _accAccel[i] = 0; }
    _accRoll = 0; _accPitch = 0;
  }

  void serviceCalibration() {
    if (_isStopped && !_isStopped()) { calFail("NOT_STOPPED"); return; }
    const uint32_t now = micros();
    if ((int32_t)(now - _calNextUs) < 0) return;
    _calNextUs = now + calSampleIntervalUs;

    PidLinkImuSample s;
    if (!_imuSample(s)) { calFail("SENSOR_FAULT"); return; }

    // Any real movement invalidates the whole average, so bail immediately
    // rather than quietly folding motion into the offsets.
    for (uint8_t i = 0; i < 3; i++) {
      if (!isfinite(s.gyro[i]) || !isfinite(s.accel[i])) { calFail("SENSOR_FAULT"); return; }
      if (fabsf(s.gyro[i]) > calMaxGyroDps) { calFail("MOVING"); return; }
    }

    if (s.hasAttitude && (!isfinite(s.roll) || !isfinite(s.pitch))) { calFail("SENSOR_FAULT"); return; }
    for (uint8_t i = 0; i < 3; i++) { _accGyro[i] += s.gyro[i]; _accAccel[i] += s.accel[i]; }
    if (s.hasAttitude) { _accRoll += s.roll; _accPitch += s.pitch; _calSawAttitude = true; }
    _calCount++;

    const uint8_t pct = (uint8_t)((uint32_t)_calCount * 100UL / calSamples);
    if (pct != _calLastPct && (pct % 10) == 0) {
      _calLastPct = pct;
      PIDLINK_STREAM.print(F("$C,PROGRESS,"));
      PIDLINK_STREAM.println(pct);
    }

    if (_calCount < calSamples) return;

    const double n = (double)_calCount;
    PidLinkCalibration c;
    c.magic = CAL_MAGIC;
    c.version = 1;
    c.flags = 0;
    for (uint8_t i = 0; i < 3; i++) {
      c.gyro[i] = (float)(_accGyro[i] / n);
      c.accel[i] = (float)(_accAccel[i] / n);
    }
    // Gravity belongs on one accel axis, so zeroing all three would be wrong.
    // Whichever axis is nearest +/-1 g keeps its 1 g and only its error is
    // taken as offset.
    uint8_t g = 0;
    for (uint8_t i = 1; i < 3; i++) if (fabsf(c.accel[i]) > fabsf(c.accel[g])) g = i;
    if (fabsf(c.accel[g]) > 0.5f) {
      c.accel[g] -= (c.accel[g] > 0 ? 1.0f : -1.0f);
    }
    if (_calSawAttitude) {
      c.roll = (float)(_accRoll / n);
      c.pitch = (float)(_accPitch / n);
    }
    c.crc = calCrc(c);

    _cal = c;
    _calAtMs = millis();
    _calState = CAL_IDLE;

    PIDLINK_STREAM.print(F("$C,OK"));
    emitOffsets();
    PIDLINK_STREAM.print(F(",saved=0"));
    PIDLINK_STREAM.println();
  }

  void calFail(const char *reason) {
    _calState = CAL_IDLE;
    PIDLINK_STREAM.print(F("$C,FAIL,"));
    PIDLINK_STREAM.println(reason);
  }

  void emitOffsets() {
    PIDLINK_STREAM.print(F(",gx=")); PIDLINK_STREAM.print(_cal.gyro[0], 4);
    PIDLINK_STREAM.print(F(",gy=")); PIDLINK_STREAM.print(_cal.gyro[1], 4);
    PIDLINK_STREAM.print(F(",gz=")); PIDLINK_STREAM.print(_cal.gyro[2], 4);
    PIDLINK_STREAM.print(F(",ax=")); PIDLINK_STREAM.print(_cal.accel[0], 4);
    PIDLINK_STREAM.print(F(",ay=")); PIDLINK_STREAM.print(_cal.accel[1], 4);
    PIDLINK_STREAM.print(F(",az=")); PIDLINK_STREAM.print(_cal.accel[2], 4);
    PIDLINK_STREAM.print(F(",roll=")); PIDLINK_STREAM.print(_cal.roll, 3);
    PIDLINK_STREAM.print(F(",pitch=")); PIDLINK_STREAM.print(_cal.pitch, 3);
  }

  void reportCalStatus() {
    PIDLINK_STREAM.print(F("$C,STATUS,valid="));
    PIDLINK_STREAM.print(calibrationValid() ? 1 : 0);
    PIDLINK_STREAM.print(F(",saved="));
    PIDLINK_STREAM.print((_cal.flags & 1) ? 1 : 0);
    PIDLINK_STREAM.print(F(",age_s="));
    PIDLINK_STREAM.print(_calAtMs ? (millis() - _calAtMs) / 1000UL : 0UL);
    emitOffsets();
    PIDLINK_STREAM.println();
  }

  static uint32_t calCrc(const PidLinkCalibration &c) {
    // Everything except the trailing crc field itself.
    const uint8_t *p = (const uint8_t *)&c;
    const size_t n = sizeof(PidLinkCalibration) - sizeof(uint32_t);
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619UL; }
    return h;
  }

  void loadCalibration() {
#if PIDLINK_USE_EEPROM
    PidLinkCalibration c;
    EEPROM.get(PIDLINK_EEPROM_ADDR, c);
    // Both guards matter: magic catches never-written EEPROM, crc catches a
    // half-written or corrupted block. A bad calibration silently applied
    // would tell the controller it is level when it is not.
    if (c.magic == CAL_MAGIC && c.crc == calCrc(c)) {
      bool sane = true;
      for (uint8_t i = 0; i < 3; i++) {
        if (!isfinite(c.gyro[i]) || !isfinite(c.accel[i])) sane = false;
      }
      if (!isfinite(c.roll) || !isfinite(c.pitch)) sane = false;
      if (sane) { _cal = c; _cal.flags |= 1; _calAtMs = millis(); }
    }
#endif
  }
};

#endif  // PIDLINK_H
