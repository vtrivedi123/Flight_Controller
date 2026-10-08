#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "AccelGuard.h"
#include "AccelDiagnostics.h"

// Fixed-width little-endian payload. Float conversion/hex formatting happens
// only in the disarmed download. Raw sensor codes already include sensor LPF1.
struct DiagnosticFrame {
  uint32_t timeUs, sampleSequence;
  uint16_t dtUs, controlUs, imuUs, radioUs, readAgeUs, freshIntervalUs;
  uint16_t flags, batteryMv, throttleUs;
  int16_t raw[6];                 // gyro XYZ (.07 dps/LSB), accel scale in SETTINGS
  int16_t pre[6];                 // calibrated, pre-software-filter: .1 dps, .001 g
  int16_t filtered[6];            // .1 dps, .001 g
  int16_t angle[3], rateTarget[3], angleTarget[2]; // .01 deg, .1 dps, .01 deg
  int16_t pid[9], lost[3];        // axis-major P/I/D; .1 virtual motor us
  uint16_t motor[4];
  int16_t stick[3];
  int16_t temperatureRaw;         // deg C = 25 + raw/256; -32768 = unavailable
  uint16_t radioAgeMs, temperatureAgeMs;
};
static_assert(sizeof(DiagnosticFrame) == 124, "Update decoder if diagnostic layout changes");
constexpr uint16_t DIAG_FRAMES = 3800; // 471200 bytes of Teensy RAM2; ~1.9 s at 2 kHz.
struct DiagnosticFastFrame {
  uint32_t timeUs;
  int16_t raw[6], filteredGyro[3], d[3];
  uint16_t flags, dtUs;
};
static_assert(sizeof(DiagnosticFastFrame)==32,"Fast decoder layout");
constexpr uint16_t DIAG_FAST_FRAMES=2048;
// RAM1 has room for this independent 1.024 s high-rate window.
static DiagnosticFastFrame diagnosticFastFrames[DIAG_FAST_FRAMES];
enum DiagnosticFlag : uint16_t {
  DF_FRESH=1, DF_ARMED=2, DF_IMU_OK=4, DF_CALIBRATED=8, DF_FAILSAFE=16,
  DF_MIX_LIMIT=32, DF_LOW_BATTERY=64, DF_RADIO_FAULT=128, DF_ANGLE_MODE=256,
  DF_CONFIG_CHANGED=512, DF_QUANTIZED_CLIP=1024, DF_SKIPPED_TICK=2048,
  DF_OVERRUN=4096, DF_CALIBRATING=8192, DF_ACCEL_CLIP=16384
};
struct DiagnosticRecorder {
  // Values are printed numerically by STATUS and understood by collector.
  enum State : uint8_t { Idle=0, Waiting=1, Recording=2, Frozen=3, Dumping=4 };
  State state=Idle;
  uint16_t count=0, divider=1, phase=0, threshold=1400;
  uint32_t waitStartMs=0, aboveStartMs=0;
  bool above=false;
  const char *reason="empty";
  uint32_t sequence=0, lastReadUs=0, freshInterval=0, tempAtMs=0;
  uint32_t captureSkipped=0, captureOverruns=0, previousSkipped=0;
  uint32_t beginUs=0, maxControlUs=0, maxDtUs=0, freshTicks=0, staleTicks=0, maxReadAgeUs=0;
  uint32_t clipTicks=0, captureId=0;
  float maxD[3]={};
  uint8_t cue=0;
  bool announceFrozen=false, sequenceCues=false;
  bool autoOwned=false, autoDumped=false, wasArmed=false, fastTriggered=false, motorSeen=false;
  uint16_t fastCount=0, fastDumpIndex=0;
  uint32_t motorStartMs=0, fastChecksum=2166136261UL;
  bool fastEndSent=false;
  int16_t raw[6]={}, temperatureRaw=INT16_MIN;
  float pre[6]={};
  uint8_t statusRegister=0;
  uint16_t dumpIndex=0;
  uint8_t dumpStage=0;
  uint32_t checksum=2166136261UL;
  AccelDiagnostics accelStats;
  bool active() const { return state==Waiting || state==Recording; }
  bool start(bool wait, uint16_t decimation, uint16_t throttle, uint32_t nowMs) {
    if (state!=Idle) return false; // An existing recording must be explicitly cleared.
    if (!(decimation==1 || decimation==2 || decimation==4 || decimation==10)) return false;
    if (wait && (throttle<1070 || throttle>1900)) return false;
    divider=decimation; threshold=throttle; phase=count=0;
    sequenceCues=wait && decimation==10;
    autoOwned=false; autoDumped=false;
    fastCount=0; fastTriggered=false; motorSeen=false;
    waitStartMs=nowMs; above=false;
    captureSkipped=captureOverruns=0;
    accelStats = AccelDiagnostics{};
    beginUs=maxControlUs=maxDtUs=freshTicks=staleTicks=maxReadAgeUs=clipTicks=0;
    maxD[0]=maxD[1]=maxD[2]=0; cue=0; announceFrozen=false; ++captureId;
    reason="running"; state=wait?Waiting:Recording;
    return true;
  }
  void stop(const char *why) {
    if (active()) { state=Frozen; reason=why; announceFrozen=true; }
  }
  void clear() {
    state=Idle; count=0; phase=0; above=false; reason="empty"; announceFrozen=false;
  }
  // Passive arming-edge observer. Never arms/disarms or changes flight inputs.
  void automatic(bool armed, bool ready, bool downloadDrained, uint32_t nowMs) {
    if (armed && !wasArmed && ready &&
        (state==Idle || (state==Frozen && autoOwned && autoDumped && downloadDrained))) {
      clear();
      start(false,10,1400,nowMs);
      autoOwned=true;
      sequenceCues=false; // Normal pilot inputs; no prescribed manoeuvre cues.
    }
    if (!armed && wasArmed && autoOwned) stop("disarmed");
    wasArmed=armed;
  }
  // Called once per control tick. No allocation, SPI, USB, or motor writes.
  bool due(uint32_t nowMs, bool armed, uint16_t throttle, bool calibrating) {
    if (state==Waiting) {
      if (nowMs-waitStartMs>=60000u) { stop("wait_timeout"); return false; }
      if (!armed || throttle<threshold || calibrating) { above=false; return false; }
      if (!above) { above=true; aboveStartMs=nowMs; }
      if (nowMs-aboveStartMs<500u) return false;
      state=Recording;
    }
    if (state!=Recording) return false;
    if (phase>0) { --phase; return false; }
    phase=divider-1;
    return true;
  }
  void committed() {
    if (++count>=DIAG_FRAMES) { state=Frozen; reason="full"; announceFrozen=true; }
  }
};
static DiagnosticRecorder diag;
DMAMEM static DiagnosticFrame diagnosticFrames[DIAG_FRAMES];

static uint16_t diagU16(uint32_t value) { return value>65535u?65535u:static_cast<uint16_t>(value); }
static int16_t diagQuant(float value, float scale, uint16_t &flags) {
  const float scaled=value*scale;
  if (!isfinite(scaled) || scaled>32767.0f || scaled<-32767.0f) {
    flags|=DF_QUANTIZED_CLIP;
    if (!isfinite(scaled)) return 0;
    return scaled>0?32767:-32767;
  }
  return static_cast<int16_t>(lroundf(scaled));
}

