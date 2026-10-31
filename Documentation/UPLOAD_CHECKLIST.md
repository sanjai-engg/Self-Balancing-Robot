# GitHub Upload Checklist

## Source and documentation

- [ ] Add/confirm the supplied sketch at `Firmware/PID_Control_Lab/PID_Control_Lab.ino`.
- [x] Identify module as ESP32-S3-WROOM-1; GPIO4–GPIO9 are GPIO-capable on this module.
- [ ] Confirm exact carrier/development-board model and header routing for GPIO4–GPIO9; do not use GPIO8/GPIO9 for SPI simultaneously with I²C.
- [x] Identify motor driver as DRV8833; author confirms breakout labels AIN1/AIN2/BIN1/BIN2 and SLP/nSLEEP.
- [x] Identify motors as N20 DC geared motors and supply as a 2S LiPo.
- [x] Record author-reported motor output wiring: OUT1/OUT2 → left motor; OUT3/OUT4 → right motor.
- [ ] Confirm physical mapping of GPIO4–GPIO7 to AIN1/AIN2/BIN1/BIN2.
- [ ] Verify the reported onboard `nSLEEP` pull-up by breakout schematic or SLP-to-GND voltage measurement; SLP header is reported externally unconnected.
- [x] Record author-reported N20 rating: 12 V, 300 RPM.
- [ ] Confirm the reported 0.3 A stall current is per motor at 12 V; add motor manufacturer/gear ratio and wheel dimensions.
- [x] Record author-reported 2S LiPo positive directly to driver motor-supply terminal and negative to driver GND.
- [ ] Confirm exact terminal silkscreen (transcribed as `ULT`, interpreted as `VM`/motor supply), battery model/capacity/discharge rating, ESP32/IMU regulation and common-ground connections.
- [ ] Add a verified schematic/wiring diagram; do not treat code pin constants as proof of physical wiring.
- [ ] Add actual Autodesk Fusion source and useful STEP/STL exports.
- [ ] Add selected photographs and a demonstration video link.
- [ ] Replace TODOs with verified facts; use “Not measured” for unavailable test results.
- [ ] Address the firmware review items before describing the build as safe/release-ready.

## Security and repository hygiene

- [ ] Check that no Wi-Fi passwords, API keys, access tokens or private credentials are committed.
- [ ] Note that the supplied firmware starts an open `PID-Lab` access point with unauthenticated motor-control endpoints; secure or disable it before uncontrolled use.
- [ ] Check photographs for private information or serial numbers that should not be public.
- [ ] Exclude build output, temporary files and unnecessary dependencies.
- [ ] Check file sizes; use Git LFS for large binary assets when appropriate.
- [ ] Review the staged file list before committing/pushing.
- [ ] Choose a license deliberately; do not add one by default if reuse terms are undecided.

## Electrical details to verify before publishing

- [ ] Add manufacturer/model and gear ratio for the 12 V, 300 RPM N20 motors; confirm the reported 0.3 A stall current basis.
- [ ] Add 2S LiPo capacity, discharge rating and model.
- [ ] Identify exact DRV8833 breakout board and document its continuous/peak current and thermal limits.
- [ ] Physically verify the reported onboard DRV8833 `nSLEEP` pull-up and logic level. The header is reported unconnected; do not assume the chip itself provides a pull-up.
- [ ] Document that 2S LiPo voltage is within the DRV8833 IC range, while 12 V exceeds the driver's 10.8 V operating maximum.
- [ ] Confirm the 12 V motors' behaviour on the lower-voltage 2S supply with measurements.
