# Lab 8 flight controller / PDB student starter resources

Package revision: 2026-10-04, native student starter with pin-redacted firmware references.

This kit supplies the native KiCad project, handout, three AI-context guides, component documentation, and project-local symbols, footprints, and 3D models. Extract the whole ZIP, then open `Lab8_Flightboard_Starter.kicad_pro` in KiCad 10. The schematic supplies components in functional blocks with no electrical connections; all resistor, capacitor, and inductor values are `TBD`. The PCB is empty. Complete the electrical design, values, footprint selection, board outline, placement, and routing yourself. This is a starterâ€”not a completed design or a submission-ready project.

Extract the entire ZIP before using its files. Markdown (`.md`) files are plain text: read them in a text editor or attach them to a file-capable AI assistant.

## Files to use

- `Lab8_Flightboard_Starter.kicad_pro`, `.kicad_sch`, and `.kicad_pcb`: project settings, unconnected component schematic, and empty two-layer/1.6 mm PCB.
- `Documentation/Lab8_Flightboard_PDB_Design_Student_Handout.docx`: the current assignment, study images, workflow, and submission requirements. This is the same current handout distributed separately. Setup instructions describe this native starter.
- `Part1_Schematic_AI_Guide.md`: context for selecting parts/values, defining nets, checking voltage domains, and allocating MCU pins once you have your own schematic.
- `Part2_PCB_AI_Guide.md`: placement, mechanics, copper, return-path, routing, and 3D-review context.
- `Part3_Submission_AI_Guide.md`: the final one-file submission and independent-extraction check.
- `Datasheets/START_HERE.md`: maps design decisions to the bundled references. `Datasheets/SOURCES.json` records each PDF's source, collection/copy date, page count, and hash.
- `libraries/README.md`: the asset inventory, import procedure, limitations, and exact IMU alternatives. The project library tables resolve from this folder.
- `Interface_Reference/README.md`: hardware/firmware interface needs and the four required ESC GPIO assignments.
- `Firmware_Reference/README.md`: two pin-redacted receiver sketches (LSM6DSO32 and MPU6050) with local headers. Only the required motor pin set remains assigned in an example channel order; these references deliberately cannot compile/flash as supplied.
- `Hardware_SPEC_CHECKLIST.md`: information still needed from your actual battery, motors, ESC harnesses, connectors, and frame.
- `PACKAGE_STATUS.json`, `PACKAGE_MANIFEST.json`, and `NATIVE_STARTER_SOURCES.json`: starter scope, file hashes, and native-file provenance. They are package records, not grades or engineering approval.

## What you are designing

The board combines flight-control electronics and battery-power distribution. Required functions include battery entry/protection, four ESC battery/ground branches, regulated 5 V and 3.3 V supplies, Teensy 4.0, radio, four motor-command/ground interfaces, battery monitoring, status indication, test access, and mounting features.

Choose ONE IMU: Adafruit LSM6DSO32 OR MPU6050. The IMU block shows U9 (Adafruit LSM6DSO32), U8 (Adafruit MPU6050), and U10 (HiLetgo/GY-521 MPU6050). Delete the two unused sensor symbols before updating the PCB. Their pin maps and mechanics are not interchangeable. Keep unused library assets if useful, but only your chosen sensor belongs in the completed schematic and board. An OLED or additional telemetry is optional unless assigned separately.

Reference components are suggestions, not a fixed completed BOM. Every student must use the exact set D4, D8, D22, and D23 for the four ESC commands, one pin per output. The motor-to-pin order is your choice; keep it consistent in your schematic, PCB and later firmware. These are Teensy GPIO labels, not symbol-pin/pad numbers. Select suitable values, ratings, packages, and compatible pins for all other interfaces yourself. No other instructor GPIO assignments, passive values or solved wiring are supplied. Firmware compilation/flashing and powered motor testing are not Lab 8 deliverables.

For mechanics, 75 mm is the suggested minimum for both width and height, and neither dimension may exceed 85 mm. The instructor's 80 x 80 mm board is an example. Four 3.3 mm non-plated holes on a centered 25 x 25 mm mounting pattern are highly recommended, not mandatory; verify the actual frame and fasteners before choosing an alternative.

## Using the assets

Use KiCad 10 unless the instructor announces a different supported version. Keep the native project files beside `sym-lib-table` and `fp-lib-table`; keep `libraries/` with them. `${KIPRJMOD}` means the directory containing that project. Open this extracted kit's projectâ€”not an instructor reference project elsewhere on your computer.

The asset pack uses local paths for its libraries and referenced models. Standard symbols/footprints used in the supplied schematic are bundled as project-local subsets, with their required models. Other standard parts can be selected from your installed KiCad libraries. This kit does not bundle the entire KiCad library or application. If you select a part absent from a bundled subset, add the needed assets or use a separately named installed library, then make your final submission portable. The starter's generic rule settings are not a validated fabrication/current specification.

Models are mechanical review aids. Some module models are simplified or shared-board-family representations. Verify actual header order, pin 1, pad numbering, dimensions, height, and cable clearance; a convincing 3D view does not prove electrical correctness.

## Working with an AI assistant

Attach the guide for your current part, your actual project files when available, the handout, and the relevant exact-part documentation. Ask for calculations, alternatives, and specific file/pin/net evidence. Verify its claims before accepting changes. Unknown current, module revision, firmware behavior, or mechanical dimensions should remain unknownâ€”not be filled in by a guess.

Do not ask the AI to recreate the instructor answer key or declare the board safe to manufacture or fly. Check your provider's privacy rules before uploading project files.

## Final submission â€” later, after completing your design

Follow the handout: submit `Lab8_Flightboard_<YourStudentID>.zip` containing the completed native project, both library tables, all used custom symbol/footprint libraries, and referenced nonstandard models. Resolve ERC/DRC errors and unconnected items, review intentional warnings, refill zones, and reopen a freshly extracted copy.

Do not submit this untouched starter, the manual, a written report, schematic PDF, separate screenshots, Gerbers, history folders, or copied instructor solutions. Keep documentation/guides as working resources; they are not required submission dependencies. Board ordering comes later, with Lab 7 and Lab 8 prepared together under the instructor's instructions.

## Expected starting checks

ERC will report unconnected/undriven pins until you complete the schematic; that is expected, not a clean electrical design. The blank PCB has no outline or components yet. Resolve these conditions through your design work rather than suppressing checks. The native starter has no inherited ERC/DRC exclusions. See `libraries/README.md` for the supplied Teensy footprint's unimplemented extra pins and model limitations. The ST LSM6DSO32 IC datasheet remains online-only; the bundled Adafruit guide does not replace it.
