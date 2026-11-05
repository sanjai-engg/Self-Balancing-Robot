# Firmware Pinout and Reported Physical Connections

This file combines GPIO definitions from the supplied Arduino sketch with wiring information reported by the author. Firmware constants are not, by themselves, proof of physical wiring.

## ESP32-S3-WROOM-1 to MPU6050

| Signal | ESP32 GPIO / value | Firmware identifier |
|---|---:|---|
| SDA | GPIO 8 | `I2C_SDA` |
| SCL | GPIO 9 | `I2C_SCL` |
| I²C address | `0x68` | `MPU_ADDR` |
| INT | Not used | — |

## ESP32-S3-WROOM-1 to DRV8833 inputs

| Firmware role | ESP32 GPIO | Expected DRV8833 input | Status |
|---|---:|---|---|
| Left motor input A | GPIO 4 | `AIN1` | Expected channel mapping; confirm physical wire |
| Left motor input B | GPIO 5 | `AIN2` | Expected channel mapping; confirm physical wire |
| Right motor input A | GPIO 6 | `BIN1` | Expected channel mapping; confirm physical wire |
| Right motor input B | GPIO 7 | `BIN2` | Expected channel mapping; confirm physical wire |

The expected A/B association follows the usual DRV8833 naming, where AIN1/AIN2 control outputs 1/2 and BIN1/BIN2 control outputs 3/4. The author reports the left motor is connected to `OUT1`/`OUT2` and the right motor to `OUT3`/`OUT4`. Check the actual input wires before treating GPIO-to-input mapping as confirmed.

## Driver power and motor outputs (author-reported)

| Driver terminal | Reported connection |
|---|---|
| Motor-supply positive | 2S LiPo positive, direct connection; terminal transcribed by author as `ULT`, interpreted as `VM` / motor-voltage input — verify printed label |
| `GND` | 2S LiPo negative; part of common ground network (confirm ESP32 and MPU6050 GND are also connected) |
| `OUT1` + `OUT2` | Left N20 DC geared motor |
| `OUT3` + `OUT4` | Right N20 DC geared motor |
| `SLP` / `nSLEEP` | No external wire attached; author reports onboard breakout pull-up, not independently verified |

## nSLEEP warning — read before wiring

The DRV8833 IC has an internal **500 kΩ pull-down** on `nSLEEP`; it does not contain an internal pull-up. A bare IC with the pin floating defaults to sleep. The breakout may have a separate onboard pull-up, as the author reports, but the exact board model/schematic is not yet known.

TI gives a minimum `nSLEEP` high threshold of 2.5 V and describes a 6.5 V internal clamp. When pulling `nSLEEP` up to `VM`, TI recommends a 20 kΩ–75 kΩ resistor. Do not connect `SLP` directly to the 2S battery positive. Verify the breakout schematic or measure `SLP` relative to GND with a high-impedance meter and the motors disconnected/secured.

Reference: [TI DRV8833 datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf), section 7.3.4.

## ESP32-S3 pin compatibility

GPIO4–GPIO9 are GPIO-capable on the ESP32-S3-WROOM-1 module, so the assignments above are compatible at the module level. Verify the exact carrier/development board brings these pins out and does not use them for onboard functions. GPIO8/GPIO9 should not simultaneously be assigned to SPI while used for I²C.

- Module: ESP32-S3-WROOM-1 (author-confirmed)
The exact carrier/development-board model and DRV8833 breakout revision have not been recorded. The onboard `nSLEEP` pull-up and the physical GPIO4–GPIO7 to AIN1/AIN2/BIN1/BIN2 input mapping have not been independently verified. Confirm these items against the actual boards and wiring before treating this pinout as a verified as-built schematic.

References: [Espressif ESP32-S3-WROOM-1 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf); [ESP32-S3-DevKitC-1 pin table](https://documentation.espressif.com/api/resource/path/docs/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html).
