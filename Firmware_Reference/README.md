# Lab 8 flight-controller firmware references

These are pin-redacted copies of the instructor's two selected receiver sketches, following Lab 7's reference-only approach. Use them to plan the hardware/firmware interface, not as ready-to-flash flight firmware. Local header dependencies are included. The original instructor sketches are unchanged.

## Required ESC pins — the same for every student

| Interface | Required Teensy 4.0 GPIOs |
|---|---|
| Four ESC commands | D4, D8, D22, D23 — use each once, in your chosen motor order |

Only this four-pin set is prescribed, not an M1–M4 assignment. Choose which motor channel uses each pin, then keep your chosen order consistent in your schematic, PCB and later firmware. The supplied sketches retain one example order; change their motor definitions to match your design during later approved firmware work. These are Teensy GPIO labels, **not** KiCad symbol-pin numbers or footprint-pad numbers; resolve those using the supplied symbol and PJRC pinout. These pins carry ESC logic commands, not motor power. Verify actual frame position, motor/propeller direction and axes during the later approved hardware procedure. The default OneShot125 path uses the motor pin definitions. The optional optimized DShot backend is tied to the example order and has a separate guard; remapping it requires adapting its register-specific backend, not merely changing constants. That is a software limitation of this example, not a Lab 8 motor-order requirement.

## Choose the reference matching your ONE installed IMU

- `Teensy_FC_LSM6DSO32_Student_Reference/`: Adafruit LSM6DSO32, using SPI in this sketch.
- `Teensy_FC_MPU6050_Student_Reference/`: MPU6050, using I2C. Either the Adafruit or HiLetgo/GY-521 module can use this sensor-family reference, but its actual breakout pinout, supply, pull-ups, address and axes must be checked independently.

Each folder contains a matching-name `.ino` and its local headers. External Arduino/Teensy libraries are not bundled. The starter schematic has three module symbols: keep the one you bought and delete the other two. You do not need both sensor families or both sketches for your design.

## What is intentionally unfinished

All nonmotor physical GPIO assignments are `UNASSIGNED_PIN = 255`: radio CE/CSN and SPI pins, IMU interface pins, battery ADC, status LED and optional debug outputs. Battery-divider resistances are zero placeholders, not design answers. The LSM diagnostic descriptor's redacted `cs_pin` is `null` and must be updated if used later.

Choose compatible pins from your own schematic and Teensy's peripheral capability map. These copies use the `SPI` instance and the MPU copy uses `Wire`; pin selection must belong to the chosen instance. If you change bus instance or sensor interface, update every affected library/object and bus call, not just a pin constant. Optional features must agree with the hardware you keep; debug PWM outputs start disabled. An omitted status output needs corresponding firmware changes, not an arbitrary dummy pin.

A deliberate Lab 8 `#error` and configuration `static_assert`s prevent accidental use. During later instructor-authorized bring-up, fill in and review the required definitions, set the actual divider values, review feature flags and metadata, then remove the reference-only `#error`. Do not bypass the guards by assigning guesses. Compilation/flashing, calibration and powered motor tests are **not Lab 8 deliverables**; firmware source is not required in the final project ZIP.

## Retained behavior is not hardware approval

The supplied copies retain the source control laws, tuning/calibration parameters, radio packets, arming/failsafe behavior and output backends. They default to OneShot125; inherited alternatives do not make every protocol appropriate for the course ESC. Review IMU range, bus rate, filtering, axes, offsets, gains, loop timing and failsafe with the actual hardware before later use. In particular, the MPU source requests 1 MHz I2C and acknowledges that this exceeds the MPU6050's specified 400 kHz limit; that inherited setting is **not an approved bus configuration**. Neither source's fitted axes nor tuning are universal for student boards.

Do not connect a LiPo/ESC/motor or install propellers as part of this design-reference task. A successful compile would establish neither flight safety nor correct electrical design.

See `DREHMFLIGHT_LICENSE.txt` and each sketch folder's attribution notice for the upstream license and origin. Source provenance and SHA-256 records are in `../NATIVE_STARTER_SOURCES.json`.
