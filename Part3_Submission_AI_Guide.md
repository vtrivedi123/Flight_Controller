# Lab 8 — Part 3: final project-ZIP AI context

This is a portability and final-design review, not a written-report assignment. The starter contains an unconnected schematic, `TBD` R/C/L values, and an empty PCB. Do not submit the untouched starter ZIP as your finished project.

## What to submit

Submit one completed `Lab8_Flightboard_<YourStudentID>.zip`, containing your `.kicad_pro`, `.kicad_sch`, `.kicad_pcb`, project `sym-lib-table` and `fp-lib-table`, every used custom `.kicad_sym`/`.pretty` library, and every referenced nonstandard 3D model.

Use project-relative custom dependencies. Do not submit the manual, a report, schematic PDF, separate screenshots, Gerbers, design notes, unrelated downloads, lock/autosave files, history repositories, or copied instructor solutions. Documentation and these guides can stay in your working resources; they are not needed in the final submission unless assigned separately. Do not impose a firmware source/build/flash requirement.

Grading is completion-based under the handout. Correct every ERC/DRC error and unconnected PCB item; review remaining warnings and confirm they are intentional. A clean rule check does not establish safe voltages, correct bus functions, adequate copper, or flight behavior.

## Final checks

1. Save the schematic and board, refill zones, close the editors, and exit KiCad so all files finish writing.
2. Confirm exactly one chosen IMU and remove unused sensor/optional footprints and circuitry. Verify that the four ESC commands use D4, D8, D22, and D23 once each at the actual Teensy GPIOs, symbol pins and footprint pads; any consistent student-selected motor order is valid. Recheck connector polarity, four command/ground paths, battery sense, rail sources, and required functions.
3. Check that custom library/model paths use `${KIPRJMOD}` and that every target file is included. A path to your Downloads/Desktop or another computer's `/Users/...` directory is not portable.
4. Run ERC and DRC on your own current project. Correct errors, inspect unconnected items and schematic–PCB parity, and understand each warning/exclusion.
5. Make the ZIP, then extract it to a different folder. Open that copy's `.kicad_pro`, not the original. Confirm symbol resolution, footprint loading, PCB opening, zone refill, 3D model loading, and rerunnable ERC/DRC.
6. Inspect module variant/pin 1, USB/cable access, screws/standoffs, IMU orientation, antenna clearance, and power-route bottlenecks one more time. Model silhouettes and zone existence are not electrical proof.
7. Submit the tested ZIP only. Keep the original working folder and a copy of the tested archive.

Prepare/order the Lab 7 RF controller and Lab 8 flightboard together later under the instructor's ordering instructions. Do not order or connect a LiPo, ESC, motor, or propeller as part of this check.

## Prompt to use with your AI

> I uploaded my final Lab 8 flight-controller/PDB KiCad ZIP, not the untouched starter. Extract it into a new isolated folder and inspect the copied project without modifying my originals. Check required files, library/model paths, exact module variants, schematic–PCB parity, ERC/DRC and exclusions, zero unconnected items, and the handout's required functions. Review power/voltage domains, MCU pin capability, one IMU choice, radio/ESC interfaces, connector/pad maps, PCB copper and return-path assumptions, and mechanical access. Give a prioritized list of confirmed errors, likely risks, and checks that are not verified, with file/pin/net/board evidence and primary citations. Missing firmware or optional OLED is not automatically incomplete. Do not change the design, supply a grade, expose instructor answers, or claim safe manufacturing or flight.
