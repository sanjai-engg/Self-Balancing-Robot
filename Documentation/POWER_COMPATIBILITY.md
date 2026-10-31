# Power Compatibility Note

## Reported project power system

- Battery: 2S LiPo. For a conventional LiPo this is typically 7.4 V nominal and 8.4 V fully charged; verify the actual pack label and chemistry.
- Motor driver: DRV8833 breakout.
- Motors: two N20 DC geared motors, author-reported 12 V and 300 RPM.
- Stall current: 0.3 A reported; still confirm whether this is the manufacturer's figure per motor at 12 V.
- Battery positive: author reports a direct connection to the breakout motor-supply input, transcribed as `ULT` and interpreted as `VM`/motor supply. Verify the exact printed label.
- Battery negative: connected to driver `GND`; author says this forms the shared common-ground network. Confirm ESP32 and MPU6050 grounds are on the same reference.

## Voltage compatibility

Texas Instruments specifies 2.7–10.8 V as the DRV8833 motor-supply operating range. A conventional 2S LiPo (7.4 V nominal; typically 8.4 V fully charged) is within that range. A 12 V supply is **outside** the DRV8833 recommended operating range and must not be applied to the driver.

If the battery is connected directly to `VM`, the 12 V N20 motors are operated below their nominal rating. Lower speed and available performance compared with 12 V operation should be expected. Do not infer actual RPM or balancing authority from a simple voltage ratio: the motor winding, load, battery sag, driver losses and chassis dynamics affect the result.

## Current compatibility: preliminary assessment

The author reports a stall current of 0.3 A. Confirm that this is the specified stall current **for each motor at 12 V**. TI's DRV8833 datasheet gives package-dependent current figures: 500 mA RMS (2 A peak) per H-bridge for the PW/TSSOP package option, and 1.5 A RMS (2 A peak) per H-bridge for PWP and RTY package options. These are IC package ratings at stated datasheet conditions, not an automatic guarantee for an unidentified breakout board.

If 0.3 A is indeed per motor at 12 V, it is below the 0.5 A RMS figure for the lowest-current package option. The DRV8833 is therefore **not obviously undersized on the reported current magnitude alone**. Final suitability still depends on the exact IC package, PCB copper/thermal design, ambient temperature, operating duty, both motors loaded simultaneously, and the validity of the motor's stated stall current. TI notes that achievable output current depends strongly on die temperature and PCB design.

## nSLEEP / SLP status and verification

The SLP/nSLEEP header is reported to have **no external wire attached**. The author believes the breakout has an onboard pull-up that holds the driver enabled. This is plausible for some breakout designs, but the exact module has not been identified and the onboard pull-up has not been verified.

Do not confuse a breakout-level pull-up with the DRV8833 chip's internal circuitry: **the IC itself has a 500 kΩ pull-down to GND**. A bare IC with nSLEEP floating defaults to sleep. TI gives a minimum high-level threshold of 2.5 V. TI also describes a 6.5 V clamp on nSLEEP and recommends a 20 kΩ–75 kΩ resistor when pulling the pin up to VM. Do not connect SLP directly to the 2S battery positive.

Verify the exact board schematic or measure SLP relative to driver GND with a high-impedance digital multimeter, with motors disconnected or mechanically restrained. Record the measured voltage and identify whether the breakout's pull-up is connected to 3.3 V, VM through a resistor, or another rail. If the pin is below 2.5 V, it is not at the specified logic-high level; if it is above 6.5 V or connected directly to the pack without a resistor, stop and inspect the circuit before operating it.

## Remaining checks

- Confirm battery chemistry, capacity, discharge rating and exact pack model.
- Confirm exact motor-supply terminal label (`ULT` as transcribed; likely a `VM`/`VMotor` marking—check the board).
- Confirm motor stall current is 0.3 A per motor at 12 V.
- Identify exact DRV8833 breakout manufacturer/model/revision and IC package.
- Verify the board-level nSLEEP pull-up and record a measured logic level.
- Verify common ground between battery/driver, ESP32 and MPU6050.
- Identify ESP32 power regulator/input and MPU6050 breakout power requirements.
- Check driver temperature during controlled tests with both motors active.

References: [TI DRV8833 datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf), especially the supply range, package current ratings, nSLEEP operation (section 7.3.4) and thermal considerations (section 10.3); [Adafruit DRV8833 breakout pinout](https://learn.adafruit.com/adafruit-drv8833-dc-stepper-motor-driver-breakout-board/pinouts) is an example of a board whose documentation describes the chip's default pull-down, not proof of the board used here.
