#pragma once
#include <math.h>

// Experimental per-axis fixed notch, Q=4, nominal sample rate 2000 Hz.
// RBJ/W3C notch equations: https://www.w3.org/TR/audio-eq-cookbook/#formulae
// Normalized coefficients are precomputed: no trig, allocation or I/O per tick.
// Direct Form I, b2=b0, a1=b1. Seed every history to the current DC value.
struct GyroNotch {
  const float b0, b1, a2;
  float x1=0, x2=0, y1=0, y2=0;
  bool primed=false;
  GyroNotch(float b0Value, float b1Value, float a2Value)
      : b0(b0Value), b1(b1Value), a2(a2Value) {}
  void reset() { primed=false; }
  float process(float input, float dtSeconds) {
    if (!isfinite(input)) { primed=false; return input; }
    // A fixed-rate filter must not ring on history from a long acquisition gap.
    if (!primed || !isfinite(dtSeconds) || dtSeconds<0.00035f || dtSeconds>0.00065f) {
      x1=x2=y1=y2=input; primed=true; return input;
    }
    const float output=b0*input+b1*x1+b0*x2-b1*y1-a2*y2;
    if (!isfinite(output)) { primed=false; return input; }
    x2=x1; x1=input; y2=y1; y1=output;
    return output;
  }
};
static GyroNotch gyroNotches[3]={
  GyroNotch(0.9216639365f, -1.3516300178f, 0.8433278731f), // roll 238 Hz
  GyroNotch(0.9031866694f, -0.9292698583f, 0.8063733389f), // pitch 328 Hz
  GyroNotch(0.9079704343f, -1.0627626032f, 0.8159408686f)  // yaw 301 Hz
};
inline void resetGyroNotches() {
  for (auto &filter : gyroNotches) filter.reset();
}


