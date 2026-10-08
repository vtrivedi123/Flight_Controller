#pragma once
#include <stdint.h>

// Reject rail values before bias removal or filtering can hide saturation.
inline bool accelSampleClipped(int16_t x, int16_t y, int16_t z) {
  return x >= 32760 || x <= -32760 ||
         y >= 32760 || y <= -32760 ||
         z >= 32760 || z <= -32760;
}


