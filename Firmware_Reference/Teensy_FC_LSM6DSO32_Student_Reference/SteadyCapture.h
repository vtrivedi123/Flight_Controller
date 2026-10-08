#pragma once
#include <stdint.h>

// Passive command-stability detector. Does not decide whether flight is safe.
struct SteadyCaptureGate {
  bool tracking=false;
  uint32_t sinceMs=0;
  int16_t low=0, high=0;
  void reset() { tracking=false; }
  bool ready(uint32_t nowMs, bool eligible, int16_t throttle,
             int16_t roll, int16_t pitch, int16_t yaw) {
    const bool centered=roll>=-30 && roll<=30 && pitch>=-30 && pitch<=30 &&
                        yaw>=-30 && yaw<=30;
    if (!eligible || throttle<1400 || !centered) { tracking=false; return false; }
    if (!tracking) { tracking=true; sinceMs=nowMs; low=high=throttle; }
    if (throttle<low) low=throttle;
    if (throttle>high) high=throttle;
    if (high-low>20) { sinceMs=nowMs; low=high=throttle; return false; }
    return static_cast<uint32_t>(nowMs - sinceMs)>=1500u;
  }
};


