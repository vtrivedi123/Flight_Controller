# Exact-hardware information to confirm

This is a working checklist, not an extra report to submit. Record assumptions privately in your project notes when useful; the handout's one-ZIP submission contract remains unchanged. This starter does not assign an unidentified battery, motor, connector or frame a made-up specification.

| Hardware | Information needed before final design |
|---|---|
| Battery | Exact chemistry/model, cell count, maximum charge voltage, capacity, permissible current, connector polarity, approved handling instructions. Standard 3S LiPo context is 11.1 V nominal / 12.6 V full-charge; other chemistry or cell count changes the checks. |
| Motors and propellers | Actual manufacturer/model, KV, voltage range, selected propeller/load data, current, mounting and CW/CCW/propeller pairing. No exact motor ratings are established by this kit. |
| ESCs | Confirm Favourite LittleBee30A-S 2–6S BLHeli_S / OneShot125, no BEC, actual battery/phase/command/return terminals, harness order and compatible logic requirements. Distributor rating is not proof of your motor's current demand. |
| Power connectors and wires | Actual current/voltage/contact ratings, wire gauge/length, polarity, pad/soldering access, conductor strain relief. |
| Teensy | Exact 4.0 module/header configuration, USB/external-power arrangement, physical pin map, height and cable insertion envelope. |
| Radio | The supplied active model is the external-antenna nRF module shown in the handout. Confirm the exact hardware variant, header map, supply transients, module body and antenna keepout; the inherited header footprint/courtyard does not establish the entire antenna envelope. |
| IMU | ONE LSM6DSO32 or MPU6050, exact breakout revision, header map, supply/logic circuitry, axis markings, bus/address configuration, height and footprint. HiLetgo/GY-521 is not automatically the Adafruit board. |
| Frame and fasteners | Suggested minimum width and height: 75 mm; maximum width and height: 85 mm. Four 3.3 mm non-plated holes on a centered 25 x 25 mm pattern are highly recommended, not mandatory. Verify actual frame fit, screw-head/standoff keepouts, stack height and component/cable clearance. |
| LEDs and passives | Selected purchasable parts, electrical/temperature/tolerance ratings, package/polarity. A model filename or footprint nickname is not a complete BOM. |
| Fabrication | Approved stackup, copper thickness, track/space/drill/edge rules, finish and assembly constraints from the current chosen process. |

No powered LiPo/ESC/motor/propeller test is requested in Lab 8. Later bring-up requires the instructor's approved procedure and hardware precautions. This checklist does not replace it.
