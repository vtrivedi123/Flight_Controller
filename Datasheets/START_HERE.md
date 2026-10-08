# Lab 8 component documentation

These references support design decisions; they are not a completed BOM, wiring solution, or passive-value answer key. `SOURCES.json` records the PDFs actually included, their original publishers/hosts, copy/download dates, page counts, and hashes. Manufacturer documents retain their own notices and revision dates. Check for updates when selecting actual parts.

## Where to start

| Decision | Bundled reference | What to verify |
|---|---|---|
| Teensy pin allocation | `PJRC_Teensy40_Pinout_Front.pdf`, `PJRC_Teensy40_Pinout_Back.pdf` | Physical GPIO and peripheral capabilities. Also read PJRC's live board documentation for voltage limits, output drive, power, VIN/VUSB, and timer groups. |
| 5 V conversion | `TI_TPS62933_Buck_Regulator.pdf` | Exact family variant, pin functions, feedback, bootstrap, soft-start/PG differences, RT/EN behavior, L/C selection, ratings, thermal conditions, and layout. |
| 3.3 V supply | `TI_LP5912_LDO.pdf` | Exact output variant/package, supply and enable, effective capacitor/ESR conditions, dropout, dissipation, and device-specific layout exceptions. |
| Inductor selection | `Bourns_SRP7028A_Inductor_Family.pdf` | Exact inductance/suffix, saturation/RMS current, DCR, body/height, pads, and regulator requirements. A `4R7` footprint candidate is not a required answer. |
| PMOS protection | `Diodes_DMP6023LE_MOSFET.pdf` | G/D/S and tab mapping, body-diode direction, gate drive/clamp, voltage/current and thermal test conditions, package. |
| Input TVS | `Littelfuse_SMBJ_TVS_Family.pdf` | Exact SMBJ17A manufacturer variant; standoff, breakdown, pulse clamp, polarity, and surge conditions. Not a perfect voltage ceiling or sustained-overvoltage switch. |
| Gate Zener | `onsemi_BZX84C_Zener_Family.pdf` | Exact BZX84C12LT1G row, tolerance, current-dependent voltage, power, polarity, and package. |
| Series Schottky | `Diodes_B340A_Schottky_Family.pdf` | Exact B340A-13-F ratings, forward drop at load, polarity, thermal conditions and pads. Determine its role/sense-point relationship in your own circuit. |
| Radio | `Nordic_nRF24L01Plus_Product_Specification.pdf` | IC supply and SPI/control behavior. The active 3D model matches the handout's external-antenna module; obtain that exact module's header, supply/current requirements, antenna and mechanical constraints separately. |
| IMU option: LSM6DSO32 | `Adafruit_LSM6DSO32_Breakout_Guide.pdf` plus the online-only [ST LSM6DSO32 IC datasheet](https://www.st.com/resource/en/datasheet/lsm6dso32.pdf) | Sensor electrical/axis/interface information plus the breakout's VIN/3Vo, header, bias/translation, host-versus-auxiliary pins, drawings and board revision. |
| IMU option: MPU6050 | `TDK_MPU6000_MPU6050_Product_Specification.pdf`, `Adafruit_MPU6050_Breakout_Guide.pdf` | Read MPU6050 sections, not MPU6000 SPI sections. Adafruit's guide applies to its board; HiLetgo/GY-521 requires its own exact pinout/mechanical verification. |
| LEDs and resistor sizing | `Dialight_SMD_LED_Selector_Guide.pdf` | Identify the exact selected color/part and obtain its detailed electrical limits. BLUE/ORANGE text and a model color do not establish forward voltage or part identity. |
| Optional OLED | `Solomon_SSD1306_IC_Datasheet.pdf` | Controller-IC reference only. It does not establish every OLED module's supply, four-pin header, onboard regulation, pull-ups or geometry. OLED is not required unless separately assigned. |

Use either LSM6DSO32 OR MPU6050, not both. A shared-family breakout guide includes other sensors; keep the IC identity distinct from shared board geometry.

## Choosing your passives

1. Identify each resistor/capacitor/inductor's job and its source/load conditions.
2. Use the relevant IC's design procedure and connected-device limits to determine candidate values or ranges.
3. For a feedback network, divider/filter, LED resistor, pull-up, series resistor or pull-down, show the equation and its assumptions; a component catalogue alone does not determine the circuit value.
4. Select an actual part with verified voltage/current/power/tolerance/temperature ratings. Check capacitor bias/ESR, inductor saturation/RMS/DCR, package, and assembly method where relevant.
5. Recheck worst conditions, physical pad mapping, and layout after selecting or substituting a part.

Generic R/C/L symbols have no mandatory purchasable part numbers. Typical application values and the instructor's values are not automatically correct for your design.

## Online sources and missing exact-hardware information

- [PJRC Teensy 4.0 board documentation](https://www.pjrc.com/store/teensy40.html).
- [Adafruit LSM board drawings](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/downloads) and [MPU6050 board drawings](https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/downloads).
- [HiLetgo MPU6050 product page](https://hiletgo.com/ProductDetail/2157948.html): supplier identification, not a guaranteed electrical schematic or drawing for every GY-521 clone.
- [PX4 OneShot125 explanation](https://docs.px4.io/v1.14/en/peripherals/oneshot) and [BLHeli_S source](https://github.com/bitdump/BLHeli/tree/master/BLHeli_S%20SiLabs): protocol/software context, not permission to flash hardware or copy another platform's pin assignments.
- [LittleBee30A-S product listing](https://speedyfpv.com/products/4pcs-favourite-littlebee-30a-esc-2-6s-with-blheli_s-firmware-oneshot125): distributor specification naming the course variant, 2–6S support and no BEC. This is not a complete manufacturer harness/thermal specification; confirm the actual hardware.
- [JLCPCB current capabilities](https://jlcpcb.com/capabilities/pcb-capabilities), or the selected approved fabricator's process documentation.

Exact battery capacity/C rating, motor model/KV/current with the selected propeller, ESC harness, power connector, wire size, and frame/fastener dimensions are not supplied by an IC datasheet. Use `Hardware_SPEC_CHECKLIST.md`; do not infer them from the reference render. No complete authoritative HiLetgo board schematic/dimension specification has been verified for this preview. Confirm its asset against the module before use.

The Nordic and optional SSD1306 PDFs are manufacturer-authored documents reused from the Lab 7 bundle, with their original hosts recorded. The remaining set uses manufacturer/breakout-vendor documents. Where the manufacturer download was unavailable, a distributor-hosted copy of the manufacturer document may be included; `SOURCES.json` records the actual download URL, not an assumed manufacturer host. If a source could not be downloaded and verified, the record identifies that limit rather than hiding it behind a fabricated PDF.

## Not available offline in this starter

- `ST_LSM6DSO32_Datasheet.pdf`: use the original URL and recorded download limitation in `SOURCES.json`.
