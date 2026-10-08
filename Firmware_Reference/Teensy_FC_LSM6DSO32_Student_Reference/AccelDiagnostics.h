#pragma once
#include <stdint.h>
#include "AccelGuard.h"

// Raw sensor-frame populations BEFORE either calibration subtraction or filtering.
// Integer accumulation avoids a sqrt/float-statistics workload in the control tick.
// Convert to physical units only on the host or during the disarmed download.
struct RawAccelPopulation {
  uint32_t count = 0;
  int64_t sum[3] = {};
  uint64_t normSquaredSum = 0;
  uint32_t normSquaredMin = UINT32_MAX;
  uint32_t normSquaredMax = 0;

  constexpr void add(const int16_t *raw, uint32_t normSquared) noexcept {
    ++count;
    for (uint8_t axis = 0; axis < 3; ++axis) sum[axis] += raw[axis];
    normSquaredSum += normSquared;
    if (normSquared < normSquaredMin) normSquaredMin = normSquared;
    if (normSquared > normSquaredMax) normSquaredMax = normSquared;
  }
};

struct AccelDiagnostics {
  // Observe the accepted +32750 cluster as well as both guard rails.
  // This threshold is passive: AccelGuard.h remains the control decision.
  static constexpr int32_t nearRailCode = 32000;
  static constexpr int32_t clipCode = 32760;
  RawAccelPopulation all, accepted, rejected;
  uint32_t positiveNear[3] = {}, negativeNear[3] = {};
  uint32_t positiveClip[3] = {}, negativeClip[3] = {};

  // Caller supplies the existing guard decision; diagnostics cannot redefine it.
  constexpr void observe(const int16_t *raw, bool clipped) noexcept {
    uint32_t normSquared = 0;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      const int64_t code = raw[axis]; // Widen before multiplication, including -32768.
      normSquared += static_cast<uint32_t>(code * code);
      if (code >= nearRailCode) ++positiveNear[axis];
      if (code <= -nearRailCode) ++negativeNear[axis];
      if (code >= clipCode) ++positiveClip[axis];
      if (code <= -clipCode) ++negativeClip[axis];
    }
    all.add(raw, normSquared);
    if (clipped) rejected.add(raw, normSquared);
    else accepted.add(raw, normSquared);
  }
};
static_assert(UINT32_MAX / 3u >= 32768u * 32768u, "Three-axis raw norm fits uint32_t");
