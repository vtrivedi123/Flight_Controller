# Lab 8 hardware/firmware interface reference

This is a planning reference. `../Firmware_Reference/` supplies two pin-redacted receiver sketches, not compiled binaries or ready-to-flash examples. Every student must use D4, D8, D22, and D23 once each for the four ESC commands. Choose the motor-to-pin order and keep it consistent across schematic, PCB and later firmware. All other compatible pin assignments are student design choices. These are Teensy GPIO labels, not KiCad symbol-pin/pad numbers. Compilation, calibration, powered bring-up and flight testing are later work, not Lab 8 deliverables.

| Function | Interface to plan | What must agree later |
|---|---|---|
| Four ESCs | D4, D8, D22, D23 in your chosen motor order: four independent logic commands with ground reference; separate battery/ground distribution | Required pin set and consistent student-selected channel order, symbol-to-pad mapping, connector order, compatible output-generation method, protocol/units and supported timer configuration. |
| nRF radio | SPI SCK/MOSI/MISO, CE and CSN; IRQ only if used | Exact module pin map, SPI instance, control GPIOs, power, later packet/settings behavior. |
| Chosen LSM6DSO32 | Supported SPI or I2C host interface, needed configuration/address and optional interrupt | Exact module, bus mode, host pins, independent CS if shared SPI, axis/frame transform. |
| Chosen MPU6050 | I2C, address configuration and optional interrupt | Exact breakout, bus/pull-up domain, address and axis/frame transform. |
| Battery measurement | Divider/filter midpoint to a compatible ADC | Actual measured source node, maximum voltage/tolerance, ADC conversion/filter and later calibration. |
| Status | Chosen output/rail indication and suitable LED current | Meaning, polarity, output pin and later active state; not a whole-system safety indicator. |
| Optional hardware | Only the interfaces you actually keep | Remove absent hardware and disable its later firmware feature. |

The course ESC is LittleBee30A-S with OneShot125. A OneShot125 command uses nominal 125–250 microsecond high pulses, not ordinary 1–2 millisecond commands. Pulse width encodes the command; it is not simply motor duty cycle, phase commutation, RPM or thrust. Correct GPIO wiring cannot establish actual pulse timing, ESC calibration, arming or failsafe. Verify these later under the approved hardware procedure. [PX4 protocol explanation](https://docs.px4.io/v1.14/en/peripherals/oneshot).

Keep the required four ESC GPIOs. Choose all other pins from PJRC's capability map and the exact supplied symbol's physical-pin-to-GPIO map. The supplied LSM reference uses SPI; the MPU reference uses I2C. A different bus instance or supported LSM interface requires corresponding later firmware changes throughout the affected calls/objects. Independent hardware channels can share a timer frequency group; frequency sharing alone is not a conflict. Document the intended supported generation method and resolve firmware/library limits later instead of assuming every pin labeled PWM is automatically equivalent.

For a conventional quad configuration, the planned motor/propeller rotation pairing includes opposite rotations. The actual channel numbering, frame axes, motor direction and propeller match must be established from the chosen hardware and later verified physically; no board screenshot certifies CW/CCW. Do not reverse battery polarity to change a brushless motor's direction.
