#pragma once
#include <inttypes.h>
#include "SteadyCapture.h"
static SteadyCaptureGate diagSteadyGate;
// Included after the flight-controller functions/globals. Diagnostic paths
// never write gains, setpoints, throttle or motor commands.
constexpr uint8_t DIAG_CONFIG_FLOATS=34;
static float diagStartConfig[DIAG_CONFIG_FLOATS], diagEndConfig[DIAG_CONFIG_FLOATS];
static bool diagConfigChanged=false;
static uint32_t diagAccelStatsChecksum=2166136261UL;

void diagSnapshot(float *out) {
  const PidGains *gains[]={&rollRatePid,&pitchRatePid,&yawRatePid,
      &rollAnglePid,&pitchAnglePid,&levelTrimDeg};
  uint8_t at=0;
  for (const auto *g:gains) { out[at++]=g->kp; out[at++]=g->ki; out[at++]=g->kd; }
  out[at++]=imuBias.gxDps; out[at++]=imuBias.gyDps; out[at++]=imuBias.gzDps;
  out[at++]=imuBias.axG; out[at++]=imuBias.ayG; out[at++]=imuBias.azG;
  const auto &c=link.calibration();
  for (uint8_t i=0;i<3;++i) out[at++]=link.calibrationValid()?c.gyro[i]:0;
  for (uint8_t i=0;i<3;++i) out[at++]=link.calibrationValid()?c.accel[i]:0;
  out[at++]=activeLevelTrimRollDeg; out[at++]=activeLevelTrimPitchDeg;
  out[at++]=DREHM_BGYRO; out[at++]=DREHM_BACCEL;
}

int diagRecord(uint32_t tickUs, uint32_t dtUs) {
  diag.automatic(armed,imuOk && imuBiasValid,usbProtocol.idle(),millis());
  const bool due=diag.due(millis(),armed,receiverCommand.throttle,
                           imuCalibrating || link.calibrationRunning());
  if (diag.state!=DiagnosticRecorder::Recording) return -1;
  if (diag.count==0) {
    captureRadioMaxUs = captureRadioCalls = captureRadioOver100Us = 0;
    captureDtMinUs = UINT32_MAX;
    captureDtOutside50Us = 0;
    diagSteadyGate.reset();
    diag.beginUs=tickUs;
    diag.previousSkipped=skippedControlTicks;
    diagSnapshot(diagStartConfig);
    diagConfigChanged=false;
  }
  diag.captureSkipped+=skippedControlTicks-diag.previousSkipped;
  const bool skipped=skippedControlTicks!=diag.previousSkipped;
  diag.previousSkipped=skippedControlTicks;
  if (dtUs < captureDtMinUs) captureDtMinUs = dtUs;
  if (dtUs < 450u || dtUs > 550u) ++captureDtOutside50Us;
  if (dtUs>diag.maxDtUs) diag.maxDtUs=dtUs;
  if (imuSampleFresh && imuOk) ++diag.freshTicks; else ++diag.staleTicks;
  const uint32_t readAge=diag.lastReadUs?micros()-diag.lastReadUs:UINT32_MAX;
  if (readAge>diag.maxReadAgeUs) diag.maxReadAgeUs=readAge;
  bool rawClip=false;
  for (uint8_t i=0;i<6;++i) if (diag.raw[i]>=32760 || diag.raw[i]<=-32760) rawClip=true;
  if (rawClip) ++diag.clipTicks;
  // Fresh samples only: a held/cached read must not reweight the population.
  if (imuSampleFresh && imuOk)
    diag.accelStats.observe(&diag.raw[3], accelSampleClipped(diag.raw[3],diag.raw[4],diag.raw[5]));
  const PidState *states[]={&rollPidState,&pitchPidState,&yawPidState};
  for (uint8_t i=0;i<3;++i) {
    const float d=fabsf(states[i]->dTerm);
    if (d>diag.maxD[i]) diag.maxD[i]=d;
  }
  // Capture only a steady-command window, not the initial throttle ramp.
  // Discard a partial snapshot if the pilot intervenes. Never change flight inputs.
  if (diag.autoOwned && diag.fastCount<DIAG_FAST_FRAMES) {
    const bool ready=diagSteadyGate.ready(millis(),
        armed && imuOk && !failsafeActive && !failsafeDescent.active() &&
        radioOk && !imuCalibrating && !link.calibrationRunning(),
        receiverCommand.throttle,receiverCommand.roll,receiverCommand.pitch,receiverCommand.yaw);
    diag.fastTriggered=ready;
    if (!ready) diag.fastCount=0;
  }
  if (diag.autoOwned && diag.fastTriggered && diag.fastCount<DIAG_FAST_FRAMES) {
    DiagnosticFastFrame &fast=diagnosticFastFrames[diag.fastCount++];
    fast.timeUs=tickUs; fast.dtUs=diagU16(dtUs);
    fast.flags=(imuSampleFresh&&imuOk?DF_FRESH:0) | (imuOk?DF_IMU_OK:0) |
        (armed?DF_ARMED:0) | (skipped?DF_SKIPPED_TICK:0) |
        (accelSampleClipped(diag.raw[3],diag.raw[4],diag.raw[5])?DF_ACCEL_CLIP:0);
    const float rates[]={imu.gxDps,imu.gyDps,imu.gzDps};
    for (uint8_t i=0;i<6;++i) fast.raw[i]=diag.raw[i];
    for (uint8_t i=0;i<3;++i) {
      fast.filteredGyro[i]=diagQuant(rates[i],10.0f,fast.flags);
      fast.d[i]=diagQuant(states[i]->dTerm,10.0f,fast.flags);
    }
  }
  if (!due) return -1;
  diagSnapshot(diagEndConfig);
  if (memcmp(diagStartConfig,diagEndConfig,sizeof(diagStartConfig))!=0) diagConfigChanged=true;
  const uint16_t index=diag.count;
  DiagnosticFrame &f=diagnosticFrames[index];
  memset(&f,0,sizeof(f)); // Includes ABI tail padding; deterministic checksum.
  f.timeUs=tickUs; f.sampleSequence=diag.sequence;
  f.dtUs=diagU16(dtUs); f.imuUs=diagU16(lastImuReadUs); f.radioUs=diagU16(radioServiceUs);
  f.readAgeUs=diagU16(readAge); f.freshIntervalUs=diagU16(diag.freshInterval);
  f.flags=(imuSampleFresh&&imuOk?DF_FRESH:0) | (armed?DF_ARMED:0) |
      (imuOk?DF_IMU_OK:0) | (imuBiasValid?DF_CALIBRATED:0) |
      (failsafeActive?DF_FAILSAFE:0) | (lastMixSaturated?DF_MIX_LIMIT:0) |
      (batteryLow?DF_LOW_BATTERY:0) | (!radioOk?DF_RADIO_FAULT:0) |
      (angleModeEnabled?DF_ANGLE_MODE:0) | (diagConfigChanged?DF_CONFIG_CHANGED:0) |
      (skipped?DF_SKIPPED_TICK:0) | (imuCalibrating?DF_CALIBRATING:0) |
      (accelSampleClipped(diag.raw[3],diag.raw[4],diag.raw[5])?DF_ACCEL_CLIP:0);
  f.batteryMv=diagU16(static_cast<uint32_t>(max(batteryVoltage,0.0f)*1000.0f));
  f.throttleUs=receiverCommand.throttle;
  const float filtered[]={imu.gxDps,imu.gyDps,imu.gzDps,imu.axG,imu.ayG,imu.azG};
  for (uint8_t i=0;i<6;++i) {
    f.raw[i]=diag.raw[i];
    f.pre[i]=diagQuant(diag.pre[i],i<3?10.0f:1000.0f,f.flags);
    f.filtered[i]=diagQuant(filtered[i],i<3?10.0f:1000.0f,f.flags);
  }
  const float angles[]={imu.rollDeg,imu.pitchDeg,imu.yawDeg};
  const float rateTargets[]={lastDesiredRollRateDps,lastDesiredPitchRateDps,lastDesiredYawRateDps};
  for (uint8_t i=0;i<3;++i) {
    f.angle[i]=diagQuant(angles[i],100.0f,f.flags);
    f.rateTarget[i]=diagQuant(rateTargets[i],10.0f,f.flags);
    f.pid[i*3]=diagQuant(states[i]->pTerm,10.0f,f.flags);
    f.pid[i*3+1]=diagQuant(states[i]->iTerm,10.0f,f.flags);
    f.pid[i*3+2]=diagQuant(states[i]->dTerm,10.0f,f.flags);
    f.lost[i]=diagQuant(states[i]->saturationError,10.0f,f.flags);
  }
  f.angleTarget[0]=diagQuant(lastCommandedRollDeg,100.0f,f.flags);
  f.angleTarget[1]=diagQuant(lastCommandedPitchDeg,100.0f,f.flags);
  for (uint8_t i=0;i<4;++i) f.motor[i]=motorOutputUs[i];
  f.stick[0]=receiverCommand.roll; f.stick[1]=receiverCommand.pitch; f.stick[2]=receiverCommand.yaw;
  f.temperatureRaw=diag.temperatureRaw;
  f.radioAgeMs=diagU16(lastValidPacketUs?(micros()-lastValidPacketUs)/1000u:65535u);
  f.temperatureAgeMs=diagU16(diag.temperatureRaw==INT16_MIN?65535u:millis()-diag.tempAtMs);
  diag.committed();
  return index;
}

bool diagParseU16(const char *s,uint16_t &value) {
  if (!s || !*s) return false;
  uint32_t n=0;
  for (;*s;++s) {
    if (*s<'0' || *s>'9') return false;
    n=n*10u+static_cast<uint32_t>(*s-'0');
    if (n>65535u) return false;
  }
  value=static_cast<uint16_t>(n); return true;
}
void diagStatus() {
  usbProtocol.print(F("$D,STATUS,state=")); usbProtocol.print(static_cast<uint8_t>(diag.state));
  usbProtocol.print(F(",count=")); usbProtocol.print(diag.count);
  usbProtocol.print(F(",divider=")); usbProtocol.print(diag.divider);
  usbProtocol.print(F(",armed=")); usbProtocol.print(armed?1:0);
  usbProtocol.print(F(",reason=")); usbProtocol.println(diag.reason);
}
void diagCommand(uint8_t n,char **fields) {
  if (n==2 && strcmp(fields[1],"STATUS")==0) { diagStatus(); return; }
  if (n==2 && strcmp(fields[1],"STOP")==0) {
    // Stop recording only. OUT,STOP is the independent motor-stop command.
    diag.stop("manual"); diagStatus(); return;
  }
  if (n==2 && strcmp(fields[1],"CLEAR")==0) {
    if (armed || diag.state==DiagnosticRecorder::Dumping) { link.error("DIAG_BUSY"); return; }
    diag.clear(); diagStatus(); return;
  }
  if (n==2 && strcmp(fields[1],"DUMP")==0) {
    if (armed || receiverCommand.throttle>THROTTLE_LOW_CUTOFF_US ||
        imuCalibrating || link.calibrationRunning()) { link.error("DIAG_DISARM_AND_LOW_THROTTLE"); return; }
    if (diag.state!=DiagnosticRecorder::Frozen || diag.count==0) { link.error("DIAG_NO_FROZEN_CAPTURE"); return; }
    diag.dumpIndex=0; diag.dumpStage=0; diag.checksum=2166136261UL;
    diag.fastDumpIndex=0; diag.fastChecksum=2166136261UL; diag.fastEndSent=false;
    diag.state=DiagnosticRecorder::Dumping;
    armButtonReleaseRequired=true;
    return;
  }
  const bool wait=n==4 && strcmp(fields[1],"WAIT")==0;
  const bool start=n==3 && strcmp(fields[1],"START")==0;
  if (wait || start) {
    uint16_t divider=0,threshold=1400;
    if (!diagParseU16(fields[2],divider) || (wait && !diagParseU16(fields[3],threshold))) {
      link.error("DIAG_BAD_NUMBER"); return;
    }
    if (imuCalibrating || link.calibrationRunning() || !imuOk) { link.error("DIAG_IMU_NOT_READY"); return; }
    if (wait && (armed || receiverCommand.throttle>THROTTLE_LOW_CUTOFF_US || !imuBiasValid)) {
      link.error("DIAG_WAIT_REQUIRES_CALIBRATED_DISARMED_LOW_THROTTLE"); return;
    }
    if (!diag.start(wait,divider,threshold,millis())) { link.error("DIAG_CLEAR_FIRST_OR_BAD_OPTIONS"); return; }
    diagStatus(); return;
  }
  link.error("DIAG_USE_STATUS_START_div_WAIT_div_thr_STOP_DUMP_CLEAR");
}

void diagPrintConfig(const char *label,const float *values) {
  usbProtocol.print(F("$D,CONFIG,")); usbProtocol.print(label);
  for (uint8_t i=0;i<DIAG_CONFIG_FLOATS;++i) { usbProtocol.print(','); usbProtocol.print(values[i],7); }
  usbProtocol.println();
}

// Each short line is FNV1a-protected INCLUDING its LF, independently of the
// existing binary-frame checksums. This runs only in the disarmed download.
void diagPrintAccelStats(uint8_t stage) {
  char line[512];
  int length = 0;
  if (stage < 3u) {
    const RawAccelPopulation *populations[] = {
        &diag.accelStats.all, &diag.accelStats.accepted, &diag.accelStats.rejected};
    const char *labels[] = {"ALL", "ACCEPTED", "REJECTED"};
    const auto &p = *populations[stage];
    if (stage == 0u) diagAccelStatsChecksum = 2166136261UL;
    length = snprintf(line, sizeof(line),
        "$D,ACCPOP,%s,n=%" PRIu32 ",sum_x=%" PRId64 ",sum_y=%" PRId64
        ",sum_z=%" PRId64 ",norm_sq_sum=%" PRIu64 ",norm_sq_min=%" PRIu32
        ",norm_sq_max=%" PRIu32 "\n", labels[stage], p.count,
        p.sum[0], p.sum[1], p.sum[2], p.normSquaredSum,
        p.count ? p.normSquaredMin : 0u, p.normSquaredMax);
  } else {
    const auto &a = diag.accelStats;
    length = snprintf(line, sizeof(line),
        "$D,ACCRAIL,near_code=%ld,clip_code=%ld"
        ",x_pos_near=%" PRIu32 ",x_neg_near=%" PRIu32
        ",y_pos_near=%" PRIu32 ",y_neg_near=%" PRIu32
        ",z_pos_near=%" PRIu32 ",z_neg_near=%" PRIu32
        ",x_pos_clip=%" PRIu32 ",x_neg_clip=%" PRIu32
        ",y_pos_clip=%" PRIu32 ",y_neg_clip=%" PRIu32
        ",z_pos_clip=%" PRIu32 ",z_neg_clip=%" PRIu32 "\n",
        static_cast<long>(AccelDiagnostics::nearRailCode),
        static_cast<long>(AccelDiagnostics::clipCode),
        a.positiveNear[0], a.negativeNear[0], a.positiveNear[1], a.negativeNear[1],
        a.positiveNear[2], a.negativeNear[2],
        a.positiveClip[0], a.negativeClip[0], a.positiveClip[1], a.negativeClip[1],
        a.positiveClip[2], a.negativeClip[2]);
  }
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) return;
  for (int i = 0; i < length; ++i)
    diagAccelStatsChecksum = (diagAccelStatsChecksum ^ static_cast<uint8_t>(line[i])) * 16777619UL;
  usbProtocol.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length));
}
void diagService() {
  // Automatically export only an automatically started capture. Wait for
  // disarm, low throttle, calibration idle and an open monitor. Preserve RAM
  // on disconnect; retry the complete dump when the monitor is available again.
  if (diag.autoOwned && !diag.autoDumped && diag.state==DiagnosticRecorder::Frozen &&
      diag.count>0 && !armed && receiverCommand.throttle<=THROTTLE_LOW_CUTOFF_US &&
      !imuCalibrating && !link.calibrationRunning() && usbProtocol) {
    diag.dumpIndex=0; diag.dumpStage=0; diag.checksum=2166136261UL;
    diag.fastDumpIndex=0; diag.fastChecksum=2166136261UL; diag.fastEndSent=false;
    diag.state=DiagnosticRecorder::Dumping;
    armButtonReleaseRequired=true;
  }
  if (diag.state==DiagnosticRecorder::Dumping) {
    if (!usbProtocol || armed || receiverCommand.throttle>THROTTLE_LOW_CUTOFF_US) {
      diag.state=DiagnosticRecorder::Frozen; return; // Retain RAM for retry.
    }
    if (!usbProtocol.canQueue(2048)) return;
    if (diag.dumpStage==0) {
      usbProtocol.print(F("$D,BEGIN,LSMD1,")); usbProtocol.print(diag.captureId);
      usbProtocol.print(','); usbProtocol.print(diag.count);
      usbProtocol.print(','); usbProtocol.print(diag.divider);
      usbProtocol.print(','); usbProtocol.print(sizeof(DiagnosticFrame));
      usbProtocol.print(F(",5.19-lsm-auto,")); usbProtocol.println(diag.reason);
    } else if (diag.dumpStage==1) {
      diagPrintConfig("START",diagStartConfig);
    } else if (diag.dumpStage==2) {
      diagPrintConfig("END",diagEndConfig);
    } else if (diag.dumpStage==3) {
      usbProtocol.print(F("$D,SETTINGS,odr_hz=3330,loop_hz=2000,spi_hz=4000000,gyro_lpf1=1,ftype=1,gyro_dps_per_lsb=0.07,acc_g_per_lsb="));
      usbProtocol.print(LSM6DSO32_ACC_G_PER_LSB,6);
      usbProtocol.print(F(",acc_fs_g=")); usbProtocol.print(LSM6DSO32_ACC_FS_G);
      usbProtocol.print(F(",acc_ctrl1_expected=")); usbProtocol.print(LSM6DSO32_ACCEL_CONFIG);
      usbProtocol.print(F(",acc_ctrl1_init_readback=")); usbProtocol.print(lsmLastAccelConfig);
      usbProtocol.print(F(",acc_stats_schema=1,acc_stats_frame=sensor_raw,acc_stats_sampling=fresh_only,experiment=acc32_user_tune,acc_lpf2=0,acc_clip_guard=1,gyro_notch_hz_roll=238,gyro_notch_hz_pitch=328,gyro_notch_hz_yaw=301,gyro_notch_q=4,gyro_notch_fs=2000,radio_schedule=post_control,fast_trigger=steady,fast_hold_ms=1500,fast_min_thr=1400,fast_span_us=20,fast_stick_limit=30,d_tau_s=0.002,roll_I_limit=135,pitch_I_limit=52.5,yaw_I_limit=150,esc_protocol="));
      usbProtocol.print(static_cast<uint8_t>(MOTOR_PROTOCOL));
      usbProtocol.print(F(",who=")); usbProtocol.print(lsmLastWhoAmI);
      usbProtocol.print(F(",last_status=")); usbProtocol.print(diag.statusRegister);
      usbProtocol.print(F(",config_changed=")); usbProtocol.println(diagConfigChanged?1:0);
    } else if (diag.dumpStage==4) {
      usbProtocol.print(F("$D,STATS,max_ctrl_us=")); usbProtocol.print(diag.maxControlUs);
      usbProtocol.print(F(",max_dt_us=")); usbProtocol.print(diag.maxDtUs);
      usbProtocol.print(F(",skip=")); usbProtocol.print(diag.captureSkipped);
      usbProtocol.print(F(",overrun=")); usbProtocol.print(diag.captureOverruns);
      usbProtocol.print(F(",fresh_ticks=")); usbProtocol.print(diag.freshTicks);
      usbProtocol.print(F(",stale_ticks=")); usbProtocol.print(diag.staleTicks);
      usbProtocol.print(F(",max_read_age_us=")); usbProtocol.print(diag.maxReadAgeUs);
      usbProtocol.print(F(",radio_max_us=")); usbProtocol.print(captureRadioMaxUs);
      usbProtocol.print(F(",radio_calls=")); usbProtocol.print(captureRadioCalls);
      usbProtocol.print(F(",radio_over_100us=")); usbProtocol.print(captureRadioOver100Us);
      usbProtocol.print(F(",min_dt_us=")); usbProtocol.print(captureDtMinUs);
      usbProtocol.print(F(",dt_outside_450_550=")); usbProtocol.print(captureDtOutside50Us);
      usbProtocol.print(F(",raw_clip_ticks=")); usbProtocol.print(diag.clipTicks);
      for (uint8_t i=0;i<3;++i) {
        usbProtocol.print(F(",max_D")); usbProtocol.print(i); usbProtocol.print('='); usbProtocol.print(diag.maxD[i],4);
      }
      usbProtocol.println();
    } else if (diag.dumpStage >= 5u && diag.dumpStage <= 8u) {
      diagPrintAccelStats(diag.dumpStage - 5u);
    } else if (diag.dumpStage == 9u) {
      usbProtocol.print(F("$D,ACCEND,")); usbProtocol.println(diagAccelStatsChecksum,HEX);
    } else if (diag.dumpIndex<diag.count) {
      // Hex avoids expensive float printing and retains a fixed, versioned layout.
      // The host validates row indexes, count, byte length and full-payload FNV1a.
      static const char hex[]="0123456789abcdef";
      char payload[sizeof(DiagnosticFrame)*2+1];
      const uint8_t *bytes=reinterpret_cast<const uint8_t *>(&diagnosticFrames[diag.dumpIndex]);
      for (size_t i=0;i<sizeof(DiagnosticFrame);++i) {
        payload[i*2]=hex[bytes[i]>>4]; payload[i*2+1]=hex[bytes[i]&15];
        diag.checksum=(diag.checksum^bytes[i])*16777619UL;
      }
      payload[sizeof(DiagnosticFrame)*2]='\0';
      usbProtocol.print(F("$D,ROW,")); usbProtocol.print(diag.dumpIndex++);
      usbProtocol.print(','); usbProtocol.println(payload);
      return;
    } else if (diag.fastDumpIndex<diag.fastCount) {
      static const char hex[]="0123456789abcdef";
      char payload[sizeof(DiagnosticFastFrame)*2+1];
      const uint8_t *bytes=reinterpret_cast<const uint8_t *>(&diagnosticFastFrames[diag.fastDumpIndex]);
      for (size_t i=0;i<sizeof(DiagnosticFastFrame);++i) {
        payload[i*2]=hex[bytes[i]>>4]; payload[i*2+1]=hex[bytes[i]&15];
        diag.fastChecksum=(diag.fastChecksum^bytes[i])*16777619UL;
      }
      payload[sizeof(DiagnosticFastFrame)*2]='\0';
      usbProtocol.print(F("$D,FAST,")); usbProtocol.print(diag.fastDumpIndex++);
      usbProtocol.print(','); usbProtocol.println(payload);
      return;
    } else if (!diag.fastEndSent) {
      usbProtocol.print(F("$D,FASTEND,")); usbProtocol.print(diag.fastCount);
      usbProtocol.print(','); usbProtocol.println(diag.fastChecksum,HEX);
      diag.fastEndSent=true; return;
    } else {
      usbProtocol.print(F("$D,END,")); usbProtocol.print(diag.captureId);
      usbProtocol.print(','); usbProtocol.print(diag.count);
      usbProtocol.print(','); usbProtocol.println(diag.checksum,HEX);
      diag.autoDumped=true;
      diag.announceFrozen=false;
      usbProtocol.println(F("$D,DONE,save this complete dump before the next run"));
      diag.state=DiagnosticRecorder::Frozen; return;
    }
    ++diag.dumpStage;
    return;
  }
  if (!usbProtocol || usbProtocol.availableForWrite()<512) return;
  if (diag.announceFrozen) {
    usbProtocol.print(F("$D,FROZEN,count=")); usbProtocol.print(diag.count);
    usbProtocol.println(diag.autoOwned?F(",recording_stopped_only; disarm for automatic download"):
        F(",recording_stopped_only; disarm then DIAG,DUMP"));
    diag.announceFrozen=false;
  }
  if (diag.state==DiagnosticRecorder::Recording && diag.count>0) {
    const uint32_t elapsed=micros()-diag.beginUs;
    uint8_t cue=1;
    if (diag.sequenceCues) cue=elapsed<4000000u?1:elapsed<8000000u?2:elapsed<12000000u?3:elapsed<16000000u?4:5;
    if (cue!=diag.cue) {
      diag.cue=cue;
      const char *labels[]={"","THROTTLE","YAW","ROLL","PITCH","CENTER"};
      usbProtocol.print(F("$D,CUE,"));
      usbProtocol.println(diag.sequenceCues?labels[cue]:"CAPTURE_STARTED");
    }
  }
}
