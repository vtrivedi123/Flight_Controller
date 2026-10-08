# Lab 8 project-local component assets

These assets accompany the native `Lab8_Flightboard_Starter.kicad_pro` project, unconnected schematic, and empty PCB. Do not import a routed instructor board as the starter.

## Import and paths

The root project-level `sym-lib-table` and `fp-lib-table` resolve these libraries with `${KIPRJMOD}` paths. Keep all folders together. Use KiCad 10 unless directed otherwise. If you use a different project directory, copy the needed libraries/models there and create equivalent project-local table entries; do not add machine-specific absolute paths.

The supplied schematic's standard symbols and assigned footprints/models are bundled under `KiCad_Standard/` as project-local subsets. Other parts are available through installed KiCad libraries. This kit does not include the entire standard library. A project subset shadows the corresponding global nickname; add an absent part to that subset or use a separately named global library. Include the exact assets needed for substitutes and make your completed project portable.

## Asset choices

| Library nickname | Purpose |
|---|---|
| `teensy` | Supplied Lab 7 Teensy symbols and Teensy40 footprint/model. Select Teensy4.0, not another device merely present in the source library. |
| `RF_Local` / `RF_Module_Local` | Lab 7 nRF symbol/header footprint with the external-antenna STEP model used in the handout's flight-controller 3D image. Verify the exact module variant and antenna envelope. |
| `IMU_Symbols` / `IMU_Modules` | Three explicit choices: `LSM6DSO32_Adafruit`, `MPU6050_Adafruit`, `MPU6050_HiLetgo_GY521`. Use ONE matching symbol/footprint pair. |
| `FC_Power` | DMP6023LE-13, SMBJ17A and BZX84C12LT1G source symbols with local footprint references. |
| `FC_Power_Footprints` | Course-local protection footprints, including the corrected official DMP6023LE pad pattern; this does not set its circuit connections. |
| `FC_Diode` / `FC_Diode_Footprints` | B340A-13-F symbol, footprint and model. |
| `FC_Inductor` / `FC_Inductor_Footprints` | SRP7028A candidate symbol/footprint and series model. Select actual inductance/ratings from your design; the source's part-specific name is not a mandatory value. |
| `FC_LED` / `FC_LED_Footprints` | Source Dialight part/0805-like package asset. It is not automatically the correct BLUE or ORANGE LED; choose exact color/electrical part and verify mapping. |
| `FC_Pads` / `FC_Mechanical` | Solder-pad options and mounting-hole footprints. Use the actual verified hole/pad geometry and intended polarity. |

## Changes limited to this package's copies

The original designs are unchanged. Copied assets have portable local references; supplier-source symbol footprint nicknames are reconciled with the tables. The radio footprint now uses the FC reference's external-antenna model and its exact offset/rotation/scale, matching the handout's 3D image. The Lab 7 symbol and numbered header pads are unchanged. The standard radio WRL remains an unassigned alternative asset, not the active model. The new starter schematic is a sanitized copy with refreshed local symbol caches; its PCB is empty. `NATIVE_STARTER_SOURCES.json` at the project root records this work.

`ASSET_SOURCES.json` records asset origins, source/output hashes, and metadata/path changes. Footprint/body transforms are retained unless that record explicitly names a change. Path resolution and pin/pad-number checks are not certification of package dimensions or correct circuit wiring.

The BLUE/ORANGE schematic instances use generic `Device:LED` symbols and the bundled standard 0805 LED footprint/model. They do not claim that the red Dialight source part is either color. Select actual purchasable LEDs and verify ratings, package, and polarity. `SCHEMATIC_ASSET_CHECKS.json` records every supplied schematic instance's symbol/footprint resolution and pin-number presence.

In these copies, duplicated interior IMU title text is removed so it no longer collides with pin names. Each module is still identified by its visible Value field. Symbol body/pin geometry and pin numbering are unchanged; the cosmetic adjustment is recorded in `ASSET_SOURCES.json`.

## Model limits

- The supplied LSM model comes from the user-provided Adafruit shared-board-family asset (`Adafruit_ISM330DHXC.STEP` in the source). Its filename is not a replacement sensor requirement; confirm the LSM6DSO32 board revision and header/body fit.
- The MPU HiLetgo/GY-521 WRL is a simplified course model. Header order and board measurements need confirmation against the actual unit; no verified full supplier board schematic is bundled.
- The active external-antenna radio STEP is the FC reference model shown in the handout. Its model transform is retained exactly; the inherited footprint outline/courtyard is not proof of the full module or antenna envelope. Verify actual module/revision, header fit, supply requirements, assembly height and antenna clearance. The unassigned standard radio WRL and Teensy header model are supplied Lab 7 visual references.
- Supplier component STEP models and standard KiCad package models aid mechanical review. A series model or colored LED body does not establish inductance, current rating, LED color/Vf, or electrical pin mapping.
- Pads and mounting holes have no standalone component body model. They do not need an invented 3D component merely to appear in a model count.

Before ordering, compare each exact module drawing/pinout to the symbol, numbered pads, orientation, and real part. Use 3D Viewer only as one part of that review.

## Lab 7 Teensy footprint limitation

The supplied `teensy:Teensy40` footprint does not implement symbol pins 45–54: GPIO 34–39, the associated GND/3V3 pads, and USB D−/D+. These are additional module connections, not ten ordinary edge-header pins. Do not allocate a required PCB connection to them using this footprint. Use verified implemented pins, or add and validate a suitable physical connection/footprint before using those signals. The source symbol and footprint geometry are retained; no invented pads have been added. `PIN_PAD_CHECKS.json` records this exception and the other symbol/pad-number comparisons.

The LSM source footprint had a premature closing parenthesis before its appended model. The package copy repairs only this structural syntax and uses a local model path; pad geometry and model transforms remain unchanged. This repair is recorded in `ASSET_SOURCES.json`.
