#pragma once

// Passive foreground health snapshots at 2 Hz, including while disarmed.
// No sensor transactions here, no controls changed, no waits or heap allocation.
// Units/schema are documented in README.md. Cached raw data needs imu_age_us.
void reportBenchHealth() {
  static uint32_t previousMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - previousMs < 500u) return;
  previousMs = nowMs;
  if (!usbProtocol || diag.state == DiagnosticRecorder::Dumping ||
      usbProtocol.availableForWrite() < 640) return;
  const uint32_t nowUs = micros();
  const unsigned long radioAge = lastValidPacketUs ?
      (nowUs-lastValidPacketUs)/1000u : UINT32_MAX;
  const unsigned long imuAge = lastFreshImuUs ? nowUs-lastFreshImuUs : UINT32_MAX;
  const unsigned long tempAge = diag.temperatureRaw == INT16_MIN ?
      UINT32_MAX : nowMs-diag.tempAtMs;
  char line[640];
  const int n = snprintf(line,sizeof(line),
      "$B,1,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu\n",
      static_cast<unsigned long>(nowMs),
      unsigned(armed),unsigned(imuOk),unsigned(imuBiasValid),
      unsigned(imuCalibrating || link.calibrationRunning()),unsigned(radioOk),
      unsigned(failsafeActive),unsigned(failsafeDescent.active()),
      unsigned(startupSafetyCleared),unsigned(armButtonReleaseRequired),
      unsigned(receiverCommand.buttons),radioAge,
      static_cast<unsigned long>(validPacketCount),
      static_cast<unsigned long>(invalidPacketCount),
      static_cast<unsigned long>(droppedSequenceCount),imuAge,
      static_cast<unsigned long>(skippedControlTicks),
      static_cast<unsigned long>(controlExecutionOverruns),
      unsigned(receiverCommand.throttle),unsigned(motorOutputUs[0]),
      unsigned(motorOutputUs[1]),unsigned(motorOutputUs[2]),unsigned(motorOutputUs[3]),
      int(diag.raw[0]),int(diag.raw[1]),int(diag.raw[2]),
      int(diag.raw[3]),int(diag.raw[4]),int(diag.raw[5]),
      int(diag.temperatureRaw),tempAge,
      lroundf(imu.rollDeg*100),lroundf(imu.pitchDeg*100),lroundf(imu.yawDeg*100),
      lroundf(levelTrimDeg.kp*100),lroundf(levelTrimDeg.ki*100),
      lroundf(lastCommandedRollDeg*100),lroundf(lastCommandedPitchDeg*100),
      static_cast<unsigned long>(usbProtocol.droppedLines()));
  if (n > 0 && static_cast<size_t>(n) < sizeof(line) &&
      usbProtocol.availableForWrite() >= n)
    usbProtocol.write(reinterpret_cast<const uint8_t *>(line),static_cast<size_t>(n));
  // Passive range/readback metadata also accompanies disarmed gravity/sign tests.
  // The existing $B,1 row remains byte-compatible. No commands or extra SPI reads.
  if (!armed && usbProtocol.availableForWrite() >= 192) {
    usbProtocol.print(F("$A,1,5.19-lsm-auto,acc_fs_g=")); usbProtocol.print(LSM6DSO32_ACC_FS_G);
    usbProtocol.print(F(",acc_g_per_lsb=")); usbProtocol.print(LSM6DSO32_ACC_G_PER_LSB,6);
    usbProtocol.print(F(",acc_ctrl1_expected=")); usbProtocol.print(LSM6DSO32_ACCEL_CONFIG);
    usbProtocol.print(F(",acc_ctrl1_init_readback=")); usbProtocol.println(lsmLastAccelConfig);
  }
}
