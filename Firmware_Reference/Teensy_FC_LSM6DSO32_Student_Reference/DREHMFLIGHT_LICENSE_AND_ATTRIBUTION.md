# dRehmFlight attribution and student-reference changes

This experimental receiver adapts the control-law structure and published starting values from Nick Rehm's dRehmFlight Teensy BETA 1.3 `controlANGLE2()` implementation:

<https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino>

The upstream repository/license terms govern the upstream material. The complete upstream GPL v3 license is included in `../DREHMFLIGHT_LICENSE.txt`. Retain applicable upstream notices when redistributing derived work.

Tailored deviations are intentional:

- The final-board nRF24 protocol, ABI packet sizes/order, STOP/arming/failsafe semantics, motor pins/order/signs and output backends are local and retained unchanged. The Tuner V2 descriptor's physical IMU chip-select metadata is redacted in this student copy.
- dRehm's normalized PID terms convert to the existing virtual-us differential mixer by `x1000`; the safer local collective-shift/differential scaling mixer replaces independent motor clamping.
- The original source has outer D commented. This experiment calculates outer measurement D, exposes it to the tuner, and defaults it to zero.
- This selected local source is `Teensy_Drone_VFinal_LSM_Accel32_UserTune`, using LSM6DSO32 over SPI. Its fitted axes, ranges, filtering, calibration and tuning are source-specific, not validated for arbitrary student boards.
- TPA is deliberately absent from the dRehm controller path. Local throttle limiting, stick shaping, 25-degree max angle, and level trim remain.

For the 2026-10-04 Lab 8 reference copy, nonmotor GPIOs and divider values were redacted, explicit student-selected SPI pin definitions were added, and deliberate compile guards were added. The four required motor GPIOs and inherited control behavior were retained. Complete modified source and local headers are provided; no binary is distributed. See `../README.md` for scope and limitations.
