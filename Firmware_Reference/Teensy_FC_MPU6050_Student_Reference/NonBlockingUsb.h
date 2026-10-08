#pragma once

#include <Arduino.h>
#include <string.h>

// Foreground-only line queue. All USB writers in the sketch must use this
// stream: otherwise another writer could consume the capacity pump() checks.
// No allocation, no USB flush/wait, and no partially dropped protocol lines.
class NonBlockingUsb : public Stream {
 public:
  static constexpr size_t QUEUE_CAPACITY = 8192;
  static constexpr size_t LINE_CAPACITY = 1024;
  static constexpr size_t REPLY_RESERVE = 6144;

  explicit NonBlockingUsb(decltype(Serial) &device) : device_(device) {}
  using Print::write;

  void begin(unsigned long baud) { device_.begin(baud); }
  explicit operator bool() const { return static_cast<bool>(device_); }
  int available() override { return device_.available(); }
  int read() override { return device_.read(); }
  int peek() override { return device_.peek(); }
  // Stream's blocking flush contract is deliberately not used by this sketch.
  void flush() override { pump(); }

  bool canQueue(size_t bytes) const {
    const size_t remaining = QUEUE_CAPACITY - queued_;
    return lineSize_ <= remaining && bytes <= remaining - lineSize_;
  }
  // Routine telemetry sees only space ABOVE the command/descriptor reserve.
  int availableForWrite() override {
    const size_t remaining = QUEUE_CAPACITY - queued_;
    const size_t freeBytes = lineSize_ < remaining ? remaining - lineSize_ : 0;
    return freeBytes > REPLY_RESERVE ? static_cast<int>(freeBytes - REPLY_RESERVE) : 0;
  }
  uint32_t droppedLines() const { return droppedLines_; }

  size_t write(uint8_t value) override {
    if (!device_) return 1;  // An absent monitor must never stall control.
    if (!discardLine_) {
      if (lineSize_ < LINE_CAPACITY) line_[lineSize_++] = value;
      else discardLine_ = true;
    }
    if (value != '\n') return 1;
    if (discardLine_ || lineSize_ > QUEUE_CAPACITY - queued_) {
      if (droppedLines_ != UINT32_MAX) ++droppedLines_;
    } else {
      const size_t tail = (head_ + queued_) % QUEUE_CAPACITY;
      const size_t first = minSize(lineSize_, QUEUE_CAPACITY - tail);
      memcpy(queue_ + tail, line_, first);
      memcpy(queue_, line_ + first, lineSize_ - first);
      queued_ += lineSize_;
    }
    lineSize_ = 0;
    discardLine_ = false;
    return 1;
  }
  size_t write(const uint8_t *data, size_t length) override {
    for (size_t i = 0; i < length; ++i) write(data[i]);
    return length;
  }

  // One bounded, capacity-checked driver write per foreground iteration.
  // Host disconnect discards the old session instead of replaying stale state.
  void pump(size_t budget = 64) {
    if (!device_) {
      head_ = queued_ = lineSize_ = 0;
      discardLine_ = false;
      droppedLines_ = reportedDrops_ = 0;
      return;
    }
    if (queued_ == 0 && lineSize_ == 0 && droppedLines_ != reportedDrops_) {
      reportedDrops_ = droppedLines_;
      print(F("$E,USB_TX_OVERFLOW,count="));
      println(reportedDrops_);
    }
    const int writable = device_.availableForWrite();
    if (writable <= 0 || queued_ == 0 || budget == 0) return;
    const size_t count = minSize(minSize(queued_, QUEUE_CAPACITY - head_),
                                 minSize(budget, static_cast<size_t>(writable)));
    const size_t written = device_.write(queue_ + head_, count);
    head_ = (head_ + written) % QUEUE_CAPACITY;
    queued_ -= written;
  }

 private:
  static size_t minSize(size_t a, size_t b) { return a < b ? a : b; }
  decltype(Serial) &device_;
  uint8_t queue_[QUEUE_CAPACITY] = {};
  uint8_t line_[LINE_CAPACITY] = {};
  size_t head_ = 0;
  size_t queued_ = 0;
  size_t lineSize_ = 0;
  bool discardLine_ = false;
  uint32_t droppedLines_ = 0;
  uint32_t reportedDrops_ = 0;
};
