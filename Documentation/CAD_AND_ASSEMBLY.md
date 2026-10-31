# CAD Design and Assembly

## 1. Design tools and file organisation

The mechanical design was created using **Autodesk Fusion**.

- `CAD/Source-Files/` — editable native Fusion design files and any linked design assets required to reopen the design.
- `CAD/Exports/` — STEP/STP exports for neutral CAD exchange and STL exports for 3D-printing workflows.

Keep the native editable source where possible. Add neutral exports so reviewers without Fusion can inspect the geometry.

## 2. Recommended deliverables

| Deliverable | Purpose | Status |
|---|---|---|
| Native Fusion source/archive | Editable source design | TODO: add actual project file |
| STEP (`.step` / `.stp`) | Neutral 3D CAD exchange | TODO: export if useful |
| STL (`.stl`) | Mesh for 3D printing | TODO: export applicable parts |
| Assembly screenshots | Explain the finished robot | TODO: add to `Media/Photos/` |
| Dimensioned drawings | Manufacturing dimensions and tolerances | TODO: add if available |

## 3. Mechanical description to complete

- Overall length × width × height: TODO
- Chassis material and thickness: TODO
- Wheel diameter and width: TODO
- Motor type: N20 DC geared motors (author-confirmed); exact manufacturer/model and mounting method: TODO
- Motor-to-wheel coupling: TODO
- ESP32 board location: TODO
- MPU6050 location and orientation relative to robot axes: TODO
- DRV8833 module location and cooling/airflow (if relevant): TODO
- Battery: 2S LiPo (author-confirmed); exact dimensions, location and retention method: TODO
- Fasteners, inserts and spacers: TODO
- Printed-part material and print settings (if 3D printed): TODO

## 4. Assembly sequence

Replace this outline with the actual procedure and add photographs:

1. Assemble the chassis and motor mounts.
2. Install the motors and wheels; verify free rotation and mechanical symmetry.
3. Mount the ESP32 and DRV8833 module with suitable insulation and mechanical support.
4. Mount the MPU6050 rigidly; record its axes relative to the robot's forward direction.
5. Secure the battery and route wires away from wheels and gears.
6. Inspect fasteners, clearances, strain relief, centre-of-mass position and access to the USB/power connectors.

## 5. Design considerations to discuss

Document the actual choices made for centre-of-mass placement, structural stiffness, sensor vibration isolation, motor-mount stiffness, access for charging/programming, protection of wiring and wheel clearance. Avoid generic claims unless they are supported by the physical design.

## 6. CAD screenshots

Add labelled images to `Media/Photos/` and link them here. Useful views include the overall isometric assembly, front, side, top, exploded view, and close-ups of the sensor and motor mounts.
