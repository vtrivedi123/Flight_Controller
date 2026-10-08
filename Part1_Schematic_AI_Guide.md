# Lab 8 — Part 1: schematic-design AI context

Attach this guide with your KiCad project, the handout, and the relevant exact-part documentation. The starter schematic has unconnected components and `TBD` R/C/L values. Complete it yourself; its starting ERC errors are expected. The handout defines the assignment; manufacturer documents define device behavior. This guide supplies review context, not solved wiring, nonmotor GPIO assignments, passive answers, or a grading rubric.

## What the flight controller must accomplish

This is the aircraft flight controller and power distribution board, not Lab 7's handheld transmitter. A Teensy 4.0 receives radio commands, reads ONE IMU and battery voltage, and provides four independent motor commands. Four ESCs take battery power from the PDB and drive the motors through their own three phase connections. The motor phase wires do not pass through the Teensy or command connectors.

The course system uses a standard 3S LiPo assumption: 11.1 V nominal and 12.6 V fully charged. Confirm the actual pack and chemistry. Use maximum pack voltage and relevant transients when checking ratings and the battery divider. The specified ESC is Favourite LittleBee 30A-S, 2–6S, BLHeli_S / OneShot125, with no BEC. Its current rating is not the actual current drawn by your motor/propeller combination. Confirm the real module and harness before assigning connector pins; an unused header position is not a proven 5 V source.

The reference architecture has input protection, a TPS62933 buck stage for 5 V, and LP5912 3.3 V supplies for logic/radio and the IMU. Recheck each exact device and load budget. Suitable substitutions are permitted. A separate IMU rail is a design decision to justify, not a requirement to copy two identical regulators regardless of your choices.

Required functions also include battery sensing, status indication, test access, and mounting. OLED/additional telemetry is optional unless separately assigned. Remove unused optional parts, footprints, and nets; account for their later firmware feature flags.

## Choose one IMU and the correct breakout

- Adafruit LSM6DSO32 supports a host SPI or I2C interface. Check the selected bus mode, configuration straps, supply/logic domain, onboard pull-ups, and axis orientation using both the ST datasheet and Adafruit board guide. The header label `SDA` also serves as SPI SDI/MOSI; distinguish host pins from auxiliary-interface pins.
- MPU6050 uses I2C; do not apply the MPU6000's SPI interface to it. Match the exact Adafruit or HiLetgo/GY-521 pin map and footprint. Adafruit's board-level regulation and translation cannot be assumed for every clone.
- One installed IMU suffices. Remove the unused sensor symbol/footprint. Interrupt and auxiliary pins need only the connections required by your selected supported configuration; polling or onboard bias may make external connections unnecessary.
- The starter shows U9 (Adafruit LSM6DSO32), U8 (Adafruit MPU6050), and U10 (HiLetgo/GY-521 MPU6050) as alternatives. Delete two before updating the PCB; do not connect all three.

Shared SPI clock/data wiring can be valid for the LSM and nRF radio when each has an independent select and compatible transaction configuration. Every student must use D4, D8, D22, and D23 for the four ESC commands, one per output; motor-to-pin order is a student choice. Different compatible pins or bus instances are allowed for the remaining interfaces. Symbol pin numbers are not automatically Teensy GPIO numbers: use the exact supplied symbol map and PJRC pinout. The pin-redacted sketches in Firmware_Reference/ expose only this required motor pin set in an example order; they deliberately cannot compile/flash as supplied.

## Questions your review should answer

- Does every supply have a known source, return, voltage domain, load budget, and suitable supporting capacitors? Do any independent regulator outputs accidentally join?
- Does the protection circuit match its exact PMOS/TVS/Zener pinout, polarity, ratings, gate drive, and intended fault conditions? Which branch is actually protected? Raw ESC branches need not flow through the logic regulator or protection FET.
- Does the buck circuit use the selected variant's VIN/GND, EN, RT, feedback, switch, bootstrap, soft-start/power-good, inductor, and capacitor requirements? Calculate its output from your chosen feedback components; identify the actual sense point and any downstream series-diode drop.
- Do the selected LDO's supply, enable, output variant, capacitors, dropout, and thermal assumptions match its documentation? Do not copy Lab 7's AP2112 requirements to LP5912.
- Are battery sense, IMU, radio, status, and four ESC commands on compatible Teensy pins? Teensy 4.0 I/O is 3.3 V and not 5 V tolerant. Check GPIO drive limits, ADC capability, bus instances, and the intended output-generation method. Hardware capability alone does not prove a OneShot waveform.
- At full battery voltage and resistor tolerances, is the divider midpoint safe for the chosen ADC input? Which node is measured—raw battery or a protected rail? The Teensy's RTC `VBAT` supply pin is not the battery ADC input.
- Do command/return and battery/return connectors match the exact external hardware? Are four channels independently controllable? Does each series resistor/pull-down serve its intended role? A pull-down is not a complete arming/failsafe system.
- Do symbol pins, footprint pads, pin 1, polarity, and breakout variant agree? Are required values selected rather than left blank/TBD? Are remaining ERC warnings understood rather than hidden?

## Prompt to use with your AI

> Open my attached Lab 8 KiCad schematic, handout, and exact-part documents. Review the power architecture, protection branches, buck/LDO topology, values and ratings, battery divider/filter, Teensy physical-pin-to-GPIO mapping, radio/IMU buses, four ESC command/ground paths, connector maps, selected IMU variant, status/test access, and ERC findings. Require D4, D8, D22, and D23 once each for ESC commands, accepting any consistent student-selected motor order; do not assume any other instructor pins, net names, values, or layout. For each concern, cite the actual component/pin/net path and manufacturer section. Separate confirmed errors, conditional recommendations, and unknowns. Explain calculations and compare candidate choices using my stated pack maximum, rail loads, and exact components so I can verify and select the final parts. Do not rewrite my design or claim it is safe to manufacture or fly.

## Before PCB layout

Confirm each required function, power/return path, safe voltage domain, passive rating, the exact ESC GPIO set D4/D8/D22/D23 and consistent chosen motor order, all other compatible pin assignments, and symbol-to-pad maps. Correct all ERC errors and review warnings. Save the resolved schematic. Later firmware definitions must match these choices; compiling or flashing firmware is not required in Lab 8.
