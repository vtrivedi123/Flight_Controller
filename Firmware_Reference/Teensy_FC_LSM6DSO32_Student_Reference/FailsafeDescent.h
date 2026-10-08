#ifndef VFINAL_FAILSAFE_DESCENT_H
#define VFINAL_FAILSAFE_DESCENT_H

#include <stdint.h>

// Open-loop RF loss response. This does not estimate altitude or guarantee a landing.
class FailsafeDescent {
 public:
  static constexpr uint32_t duration_us = 3000000UL;

  bool active() const { return active_; }

  void start(uint32_t now_us, int16_t throttle_us) {
    active_ = true;
    start_us_ = now_us;
    start_throttle_us_ = throttle_us;
    last_throttle_us_ = throttle_us;
  }

  void cancel() { active_ = false; }

  // Returns false at the low-throttle endpoint; caller must disarm immediately.
  bool advance(uint32_t now_us, int16_t low_throttle_us, int16_t &throttle_us) {
    if (!active_) return false;
    const uint32_t elapsed_us = now_us - start_us_;  // Unsigned wrap is intentional.
    if (elapsed_us >= duration_us) {
      throttle_us = low_throttle_us;
      active_ = false;
      return false;
    }
    const int32_t span_us = static_cast<int32_t>(start_throttle_us_) - low_throttle_us;
    const int32_t reduction_us = static_cast<int32_t>(
        (static_cast<uint64_t>(span_us) * elapsed_us) / duration_us);
    int16_t next_us = static_cast<int16_t>(start_throttle_us_ - reduction_us);
    if (next_us > last_throttle_us_) next_us = last_throttle_us_;
    if (next_us < low_throttle_us) next_us = low_throttle_us;
    last_throttle_us_ = next_us;
    throttle_us = next_us;
    return true;
  }

 private:
  bool active_ = false;
  uint32_t start_us_ = 0;
  int16_t start_throttle_us_ = 0;
  int16_t last_throttle_us_ = 0;
};

#endif


