# Bill of Materials (BOM)

This working BOM is based on the supplied firmware and author-confirmed physical hardware. Items marked TODO must be filled from the actual parts or their datasheets.

| Qty | Item | Identified specification | Remaining details |
|---:|---|---|---|
| 1 | Controller module | ESP32-S3-WROOM-1 | Carrier/development-board model and revision: TODO |
| 1 | IMU breakout | MPU6050 | Breakout model and supply voltage: TODO |
| 1 | Dual H-bridge motor driver | DRV8833 breakout | Manufacturer/model/revision, IC package, thermal design and onboard `nSLEEP` pull-up: TODO |
| 2 | Geared DC motor | N20, 12 V, 300 RPM | Manufacturer, gear ratio; 0.3 A stall current reported—confirm per motor at 12 V |
| 2 | Wheel | Two-wheel differential drive | Diameter, width, material and hub: TODO |
| 1 | Battery | 2S LiPo; usually 7.4 V nominal and 8.4 V fully charged for standard LiPo chemistry | Exact model, capacity, discharge rating and chemistry: TODO |
| 1 | Chassis | Autodesk Fusion design | Material, dimensions and fabrication method: TODO |
| — | Wiring and connectors | As built | Wire gauge, connectors, switch, fuse and power distribution: TODO |

## Author-reported wiring

- 2S LiPo positive is connected directly to the DRV8833 motor-supply terminal, transcribed by the author as `ULT` and interpreted as `VM`/motor voltage. Confirm the exact silkscreen; `VM` is the standard DRV8833 IC name.
- 2S LiPo negative is connected to driver `GND` as part of the common-ground network. Confirm controller and IMU grounds join this reference.
- Left N20 motor is connected to `OUT1`/`OUT2`.
- Right N20 motor is connected to `OUT3`/`OUT4`.
- The `SLP`/`nSLEEP` header has no external wire. The author reports an onboard breakout pull-up; this is not verified because the breakout model is unknown.

## Voltage and current checks

- TI specifies the DRV8833 motor-supply operating range as 2.7–10.8 V. A standard 2S LiPo (7.4 V nominal, typically 8.4 V fully charged) is within the IC range. A 12 V supply is outside the recommended operating range and must not be applied to the DRV8833.
- The 12 V-rated motors are operated below their nominal voltage when powered directly from the 2S pack. Expect lower speed/available performance than at 12 V; do not infer exact speed from voltage ratio alone.
- The author reports 0.3 A stall current. Confirm that this is the manufacturer's stall-current value **per motor at 12 V**, not a combined or running-current figure. TI lists package-dependent continuous RMS output-current figures: 0.5 A per H-bridge for the PW/TSSOP option and 1.5 A per H-bridge for PWP/RTY options. The exact module IC/package and breakout thermal design remain unknown. On the reported current alone the driver is not obviously undersized, but this is not a thermal sign-off.
- The DRV8833 IC has an internal 500 kΩ pull-down on `nSLEEP`. A bare IC with `nSLEEP` open defaults to sleep. If the board works with its SLP header externally unconnected, the breakout must have an onboard pull-up or another board-level path; identify/measure it. TI recommends 20 kΩ–75 kΩ if `nSLEEP` is pulled up to `VM`, due to the 6.5 V internal clamp. Do not connect `SLP` directly to the 2S pack.
- Power the ESP32 and MPU6050 using suitable regulated rails. Do not connect the 2S pack directly to a 3.3 V rail. Keep controller, IMU and driver grounds common where required by the circuit.

## References

- [Texas Instruments DRV8833 datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [TI nSLEEP operation](https://www.ti.com/lit/ds/symlink/drv8833.pdf) (section 7.3.4)
- [Adafruit DRV8833 breakout pinout](https://learn.adafruit.com/adafruit-drv8833-dc-stepper-motor-driver-breakout-board/pinouts) (example board only; do not assume it is the same breakout)
