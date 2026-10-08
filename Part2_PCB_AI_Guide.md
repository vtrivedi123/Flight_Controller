# Lab 8 — Part 2: PCB-design AI context

Attach this guide with your schematic, PCB, footprint libraries, models, relevant component layout guidance, and DRC output. The supplied PCB is empty: no instructor placement, outline, nets, tracks, vias, or zones are provided. Complete the schematic first, choose one IMU, update the PCB, then design its mechanics and layout. Inspect actual project files; screenshots alone cannot establish net connectivity or copper dimensions. A clean DRC and plausible 3D image do not prove a functioning board.

## Scope and design freedoms

The board must distribute battery power to four ESCs while keeping logic, radio, IMU, and ADC interfaces usable. Suitable component substitutions, package sizes, and compatible nonmotor pin maps are allowed. The four ESC commands must use D4, D8, D22, and D23, one pin per output, in your chosen motor order (Teensy GPIOs, not symbol-pin/pad numbers). Recheck electrical ratings, pin/pad mapping, and assembly fit for every change. Use two copper layers and 1.6 mm board thickness unless the instructor approves another stackup and its cost; confirm actual copper thickness with the fabricator.

Use exactly ONE LSM6DSO32 or MPU6050 footprint matching your sensor. The asset directory may contain alternatives; the completed board must not contain both unused sensor footprints. OLED/additional telemetry is optional; battery sensing, status, and test access are required functions.

## Mechanical reference

The suggested minimum is 75 mm for both board width and height; neither dimension may exceed 85 mm. The 80 x 80 mm instructor board is a reference example, not a required exact size. Four 3.3 mm non-plated mounting holes on a centered 25 x 25 mm pattern are highly recommended, not mandatory. Confirm any alternative mounting arrangement against the actual frame and fasteners.

The handout records an 80 x 80 mm reference board with 1 mm-radius corner fillets and center `(40, 40)` mm measured from the upper-left bounding corner. Four 3.3 mm non-plated holes have centers `(27.5, 27.5)`, `(52.5, 27.5)`, `(27.5, 52.5)`, and `(52.5, 52.5)` mm: a centered 25 x 25 mm pattern, with X right and Y down.

Those hole coordinates apply to the 80 x 80 mm example only; if you use the recommended pattern on another board size, recenter it rather than copying absolute coordinates. Verify the actual frame, standoffs, screw heads, board height, cables, and installation space before fixing Edge.Cuts. Keep perimeter ESC power/ground pads symmetric using the approved Motor 2 edge-setback reference. No numeric setback is supplied here: establish it from the course mechanical reference rather than inventing one. Distinguish pad-center setback from copper-to-edge clearance, especially near curved corners.

Keep USB insertion and programming access clear. Identify forward direction and record IMU axes relative to the frame. A shared-family or simplified 3D model is a physical aid, not proof of exact module height or pin order.

## Placement sequence to review

1. Update the board from your completed schematic. Resolve missing footprints, duplicate references, and library problems before placement.
2. Establish the closed outline, mounting holes, mechanical keepouts, and required cable access.
3. Place battery/ESC power interfaces for practical soldering, cable direction, polarity labeling, conductor size, and strain relief.
4. Place the Teensy for USB access and the chosen IMU with deliberate axes, mounting height, and separation from noisy/high-current regions.
5. Group the protection and each regulator's support parts using its exact datasheet layout guidance. Control power-loop area, switch-node routing, and feedback paths.
6. Place the radio for its exact antenna clearance, local decoupling, and module/header fit. The supplied active radio model is the external-antenna module shown in the handout, using the FC reference's model transform. Confirm the actual variant and full antenna/cable envelope; the inherited footprint courtyard alone is not sufficient.
7. Keep battery sensing, status indicators, and test features accessible, understandable, and clear of screws, module bodies, and cable insertion envelopes.

## Routing questions your review should answer

- Which copper carries combined battery current, and which carries one ESC branch or a regulated load? Trace both positive and return paths. An ESC's 30 A label does not establish actual branch demand.
- Are widths and bottlenecks checked using stated load, copper thickness, allowable temperature rise, layer, path length, pads, vias, and thermal spokes? Do not copy one numeric width to every net. Missing assumptions mean current capacity is not verified.
- Are each buck's input loop, switch node, inductor, output return, and feedback routing arranged according to its documentation? Proximity checks are heuristics, not EMI/stability simulations.
- Are capacitor connections and layout assessed per device? LP5912 has documented remote-output-capacitor arrangements; do not invent a universal rule that every LDO output capacitor must be adjacent. Check the exception's conditions, input-capacitor guidance, and effective capacitance.
- Are radio and IMU signals compatible with the intended buses and command outputs? Is the ADC path kept away from noisy copper with a sensible return?
- Are zone nets, fill state, islands, thermals, mounting clearances, and antenna keepouts intentional? A ground-zone declaration does not establish a good return path. Valid top/bottom plane arrangements should not be rejected merely for differing from the reference.
- Do real pad numbers and nets match the schematic after updating? Is every required connection routed? Do models, headers, module bodies, screws, probes, and cables fit?
- Do clearances, drills, copper-to-edge distances, and stackup meet the selected fabricator's current published process? A model is not fabrication approval.

## Prompt to use with your AI

> Open my attached Lab 8 schematic and PCB, project libraries, 3D models, and DRC report. If placement is unfinished, propose a placement sequence and groups/orientations; otherwise critique the actual positions. Review mounting/outline, symmetric power pads, USB/cable access, IMU axes and noise exposure, regulator loops and device-specific capacitor guidance, exact radio antenna area, power/return copper, bottlenecks, vias, zones, ADC routing, connector mapping, unrouted items, and model identity. State the current/copper/temperature and mechanical assumptions behind every calculation. Cite board items/nets/coordinates and the manufacturer guidance supporting concerns. Separate errors from heuristics and unknowns. Do not copy the instructor layout, invent current demand, or declare the board manufacturing- or flight-ready.

## Before final packaging

Refill zones, inspect the filled copper, correct DRC errors, rerun DRC, and confirm zero unconnected items and schematic–PCB parity. Review intentional warnings. Inspect 3D access/alignment as well as electrical mappings. No battery/ESC/motor powered test or fabrication order is part of this resource-package check.
