# Hardware and Wiring

> This document distinguishes firmware-defined pins from physical wiring reported by the project author. The physical wiring details below are author-reported; they have not been independently inspected from a board photograph or schematic.

## 1. Bill of materials

| Item | Identified information | Still required |
|---|---|---|
| Microcontroller | ESP32-S3-WROOM-1 module; source comments specify ESP32 Arduino core 3.x | Exact carrier/development-board model and revision |
| IMU | MPU6050, I²C address `0x68` | Breakout model, supply connection, mounting orientation |
| Motor driver | DRV8833 dual H-bridge breakout | Exact breakout manufacturer/model/revision, IC package and thermal/current design |
| Motors | Two N20 DC geared motors, 12 V, 300 RPM | Manufacturer/model, gear ratio; confirm 0.3 A stall current is per motor at 12 V |
| Battery | 2S LiPo; positive reported connected directly to driver motor-supply terminal | Exact pack model, capacity, discharge rating, chemistry and protection details |
| Wheels | Two-wheel drive | Diameter, width, material and hub/coupler |
| Chassis | Designed in Autodesk Fusion | Material, dimensions, thickness and fasteners |

## 2. MPU6050 to ESP32

| MPU6050 signal | ESP32 GPIO / value in firmware | Notes |
|---|---:|---|
| SDA | GPIO 8 | `Wire.begin(8, 9)` |
| SCL | GPIO 9 | I²C clock configured to 400 kHz |
| I²C address | `0x68` | Firmware uses this fixed address |
| VCC | TODO | Check the exact breakout's input-voltage requirements; do not infer from the bare IC alone |
| GND | Common ground required | Confirm connection to ESP32/driver ground |
| INT | Not used in sketch | No interrupt pin is configured |

The firmware reads 14 bytes beginning at MPU6050 register `0x3B`. It does not use an external MPU6050 library.

## 3. ESP32 to DRV8833 control inputs

The sketch defines these signals:

| Firmware signal | ESP32 GPIO | Firmware role |
|---|---:|---|
| `PWM_LA` | GPIO 4 | Left motor H-bridge input A |
| `PWM_LB` | GPIO 5 | Left motor H-bridge input B |
| `PWM_RA` | GPIO 6 | Right motor H-bridge input A |
| `PWM_RB` | GPIO 7 | Right motor H-bridge input B |

The author reports that the left motor is connected to driver outputs `OUT1` and `OUT2`, and the right motor to `OUT3` and `OUT4`. For the standard DRV8833 channel mapping, outputs 1/2 are controlled by `AIN1`/`AIN2`, and outputs 3/4 are controlled by `BIN1`/`BIN2`. The expected corresponding input wiring is therefore:

| ESP32 GPIO | Expected driver input | Motor output channel | Status |
|---:|---|---|---|
| GPIO 4 | `AIN1` | `OUT1`/`OUT2` — left motor | Expected; verify physical wire |
| GPIO 5 | `AIN2` | `OUT1`/`OUT2` — left motor | Expected; verify physical wire |
| GPIO 6 | `BIN1` | `OUT3`/`OUT4` — right motor | Expected; verify physical wire |
| GPIO 7 | `BIN2` | `OUT3`/`OUT4` — right motor | Expected; verify physical wire |

The motor output pairing is author-reported. The exact input-to-GPIO wires are inferred from the code's left/right naming and standard DRV8833 channel mapping; check them against the actual wires before making a final schematic.

## 4. Motor driver power and outputs — author-reported physical wiring

| Connection | Reported wiring | Status / note |
|---|---|---|
| Battery positive | Directly to the breakout motor-supply input; author transcribed the silkscreen as `ULT` and identified it as `VM` / motor voltage | Check exact printed label. Standard DRV8833 IC pin name is `VM`; breakout boards may label it `VM`, `VMotor`, or similar. |
| Battery negative | To driver `GND` | Author-reported |
| Common ground | Battery negative/driver GND forms part of the common-ground network | Confirm ESP32 GND and MPU6050 GND are tied to the same reference |
| Left N20 motor | Driver `OUT1` and `OUT2` | Author-reported |
| Right N20 motor | Driver `OUT3` and `OUT4` | Author-reported |
| SLP / nSLEEP header | No external wire attached | Author reports the breakout has an onboard pull-up; this must be verified against the exact board or by measurement |

### Important clarification about SLP / nSLEEP

The **DRV8833 IC itself has an internal 500 kΩ pull-down** from `nSLEEP` to GND. It does not have an internal pull-up. A bare IC with `nSLEEP` left unconnected therefore defaults to sleep. TI states that `nSLEEP` must be logic-high to enable the H-bridges, and gives a minimum high-level threshold of 2.5 V.

The author's explanation can still be correct at the **breakout-board level**: a board-level pull-up component/network may hold `nSLEEP` high even though the header is not externally wired. However, the exact breakout board has not been identified, and the alleged onboard pull-up has not been confirmed from a schematic or measurement. Record it as **author-reported / verify**, not as a verified property of the DRV8833 IC.

Suggested verification:

1. Disconnect the motors or secure the robot so the wheels cannot drive it unexpectedly.
2. Identify the exact breakout board and inspect its schematic or visible pull-up component.
3. With the driver powered and using a high-impedance digital multimeter, measure `SLP` relative to driver `GND`.
4. A reading at or above 2.5 V is a logic-high level according to TI; a reading at or below 0.5 V is logic-low. If the voltage is not clearly in either range, investigate before use.
5. Do not connect `SLP` directly to the 2S battery positive. TI describes a 6.5 V internal clamp and recommends a 20 kΩ–75 kΩ resistor when pulling `nSLEEP` up to `VM`, because an unresisted supply connection can damage the input structure. A known 3.3 V logic pull-up may be suitable if allowed by the exact board design.

Reference: [TI DRV8833 datasheet, nSLEEP operation](https://www.ti.com/lit/ds/symlink/drv8833.pdf).

## 5. ESP32-S3-WROOM-1 pin compatibility

The confirmed module is **ESP32-S3-WROOM-1**. GPIO4, GPIO5, GPIO6, GPIO7, GPIO8 and GPIO9 are GPIO-capable on this module, so the sketch's pin assignments are compatible at the module pin-function level. This is different from the original ESP32-WROOM-32, where GPIO6–GPIO11 are connected to the SPI flash interface.

The exact carrier/development-board model is still unknown. Confirm that its headers expose GPIO4–GPIO9 and that onboard circuitry does not use them. GPIO8 and GPIO9 have alternate SPI-related functions; do not assign those pins to SPI at the same time as this sketch's I²C bus.

References: [Espressif ESP32-S3-WROOM-1 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) and [ESP32-S3-DevKitC-1 official header pin table](https://documentation.espressif.com/api/resource/path/docs/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html).

## 6. MPU6050 firmware configuration

| Setting | Register value / firmware configuration | Meaning |
|---|---|---|
| Power management | `PWR_MGMT_1 = 0x01` | Wake device and select PLL clock source |
| DLPF | `CONFIG = 0x03` | Source comment identifies 44 Hz bandwidth |
| Sample divider | `SMPLRT_DIV = 0x04` | 200 Hz output rate with configured DLPF |
| Gyro full scale | `GYRO_CONFIG = 0x08` | ±500 °/s; code uses 65.5 LSB/(°/s) |
| Accelerometer full scale | `ACCEL_CONFIG = 0x08` | ±4 g; code scales readings using 8192 LSB/g |
| I²C clock | 400 kHz | `Wire.setClock(400000)` |

The sketch chooses gyro Y as the pitch axis (`PITCH_GYRO_AXIS = 1`), applies gyro sign −1, and computes accelerometer tilt from X/Z with sign +1 by default. At boot, it averages 400 pitch-axis gyro samples while the robot is held still to estimate `gyroOffset`. The dashboard's **Set upright zero** action records an accelerometer angle trim in ESP32 Preferences storage.

## 7. Motor output configuration

- PWM frequency at startup: 20,000 Hz.
- PWM resolution: 8 bits (duty range 0–255).
- Default motor deadband compensation: 30 counts.
- Deadband ramp window: 10 command counts.
- Left motor sign: +1; right motor sign: −1.
- Command range before motor output: −255 to +255.

The sketch also allows PWM frequency and deadband to be changed through its serial command interface/dashboard. Verify motor direction with the robot safely raised before balancing tests.

## 8. Power architecture and unresolved details

The author reports that the 2S LiPo positive terminal connects directly to the driver motor-supply terminal and battery negative connects to driver `GND`. A standard 2S LiPo is 7.4 V nominal and typically 8.4 V fully charged; verify the actual pack label and chemistry. The DRV8833 IC motor-supply operating range is 2.7–10.8 V, so this supply is within the IC range. The 12 V N20 motors are being supplied below their reported rated voltage.

Still confirm:

- Exact battery model, capacity, C/discharge rating and chemistry.
- Exact silkscreen on the motor-supply terminal (reported as `ULT`, interpreted as `VM`/motor voltage).
- ESP32 power source/regulator and input pin.
- MPU6050 breakout supply voltage and regulator/level shifting, if any.
- Explicit shared-ground connection from driver/battery to ESP32 and MPU6050.
- Main power switch, fuse and low-voltage cutoff/protection.
- Exact DRV8833 breakout manufacturer/model/revision and IC package.
- Onboard `nSLEEP` pull-up circuit; external header is reported as unconnected.
- Whether the 0.3 A stall current is specified per motor at 12 V.

Do not power the motors from the ESP32 3.3 V pin. Disconnect the battery before rewiring. Consult the exact driver-board documentation and test thermal performance under controlled load.

## 9. Motor and mechanical details

- Motor type: N20 DC geared motor.
- Rated voltage: 12 V (author-reported).
- Speed: 300 RPM (author-reported; confirm whether no-load gearbox output speed).
- Stall current: 0.3 A reported; confirm per motor at 12 V.
- Manufacturer/model and gear ratio: TODO.
- Wheel diameter/width/material: TODO.
- Chassis material/dimensions and fasteners: TODO.
- Sensor mounting orientation and position: TODO.

## 10. Wiring diagram and photographs

Add a verified schematic or wiring photograph under `Hardware/` and link it here. Do not use a diagram that merely repeats firmware GPIO assignments without confirming the physical connections.
