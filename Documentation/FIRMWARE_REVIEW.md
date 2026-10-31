# Firmware Review Notes

This review is based on the supplied `PID_Control_Lab.ino`. It distinguishes implemented behaviour from issues that should be addressed before describing the firmware as robust or safe.

## Confirmed from the code

- ESP32 Arduino core 3.x is stated in the hardware comments; the module used is ESP32-S3-WROOM-1.
- MPU6050 at I²C address `0x68`; SDA GPIO 8, SCL GPIO 9.
- DRV8833 motor driver is explicitly named in the source hardware comments; author confirms the breakout has AIN1/AIN2/BIN1/BIN2 and SLP/nSLEEP labels.
- Author confirms two 12 V, 300 RPM N20 DC geared motors and a 2S LiPo battery (7.4 V nominal; typically 8.4 V fully charged). The author reports 0.3 A motor stall current; whether this is per motor at 12 V must be confirmed. Motor manufacturer and battery capacity/discharge rating remain unknown.
- Left motor input pair: GPIO 4/5; right motor pair: GPIO 6/7.
- Two-state Kalman estimator tracks angle and gyro bias.
- Single PID angle loop runs at a nominal 100 Hz on FreeRTOS core 1.
- Default derivative is taken from the gyro measurement (`-gyroRate`).
- Firmware has no encoder feedback, steering, speed loop, or position loop.
- The ESP32 creates an open `PID-Lab` Wi-Fi access point and provides an unauthenticated control dashboard.

## Issues to address or explicitly document

### 1. IMU read failure does not fail safe

In `controlLoop`, `if (!readIMU()) continue;` skips the rest of the iteration after an I²C failure. It does not disarm or call `stopMotors()` first. If the motors were being driven, the previously written PWM duty can remain active.

**Recommended correction:** On a failed IMU read, disarm balancing, clear the integral, clear any motor test command, command zero to all four PWM outputs, and then continue. Test this deliberately by disconnecting or interrupting the IMU bus with the robot mechanically restrained and the wheels clear of the floor.

### 2. Open Wi-Fi access point and unauthenticated control endpoints

The sketch calls `WiFi.softAP("PID-Lab", "")`. The web server endpoints can start/stop motors, change PID gains, set motor test output and alter sensor-axis settings. There is no authentication in the supplied code.

**Recommended correction:** Add authentication and a non-empty AP password, or disable the AP outside controlled bench work. Do not connect this control AP to an untrusted environment.

### 3. ESP32-S3-WROOM-1 GPIO mapping: module-level check passed; carrier board still to verify

The project uses an **ESP32-S3-WROOM-1** module. Espressif's module pin definitions list GPIO4–GPIO9 as GPIO-capable, so the sketch's GPIO4–GPIO7 motor outputs and GPIO8/GPIO9 I²C assignments are compatible at the module level. The previous GPIO6–GPIO11 SPI-flash warning applies to the original ESP32-WROOM-32, not to this confirmed S3 module.

The exact carrier/development-board model is still unknown. Confirm that its header layout exposes these pins and that onboard peripherals do not occupy them. GPIO8/GPIO9 also have alternate SPI-related functions, so they must not be used for SPI concurrently with this sketch's I²C bus.

### 4. DRV8833 `nSLEEP` is not controlled by the sketch; breakout pull-up is reported but unverified

There is no GPIO assignment or explicit write for `nSLEEP`. The author now confirms that the breakout's SLP header has **no external wire attached** and reports that the board has an onboard pull-up which holds the driver enabled. This has not been verified against the exact breakout schematic or by measurement because the breakout manufacturer/model is still unknown.

Important distinction: the DRV8833 IC itself has an internal **500 kΩ pull-down**, not a pull-up. A bare IC with `nSLEEP` open defaults to sleep. If the robot operates with no external SLP wire, a board-level pull-up or another board-level electrical path must be present. TI specifies a minimum `nSLEEP` logic-high threshold of 2.5 V. TI also describes a 6.5 V internal clamp and recommends a 20 kΩ–75 kΩ resistor when pulling `nSLEEP` up to `VM`. Do not connect SLP directly to the 2S pack. Identify the breakout and verify SLP-to-GND voltage with a high-impedance meter (motors disconnected or restrained). [TI DRV8833 datasheet, section 7.3.4](https://www.ti.com/lit/ds/symlink/drv8833.pdf).

The author reports battery positive connected directly to the breakout motor-supply terminal (silkscreen transcribed as `ULT`, interpreted as `VM`/motor voltage) and battery negative connected to driver `GND`. The author reports `OUT1`/`OUT2` to the left motor and `OUT3`/`OUT4` to the right motor. Confirm the exact supply-terminal silkscreen and that ESP32/MPU6050 grounds share the same reference.

### 5. Motor voltage and current compatibility: preliminary assessment

The author reports N20 motors rated 12 V, 300 RPM and a stall current of 0.3 A. If this means **0.3 A per motor at 12 V**, it is below the lowest continuous RMS/DC current figure listed by TI for DRV8833 package options (0.5 A per bridge for the PW/TSSOP option). On the reported current alone, the DRV8833 is not obviously undersized. Confirm that the figure is per motor and manufacturer-specified at 12 V (rather than a running current or combined current), and identify the exact IC package and breakout-board revision. Board thermal performance and layout still matter, particularly with both channels operating.

The 2S LiPo (7.4 V nominal; typically 8.4 V fully charged for standard LiPo chemistry) is within the DRV8833 2.7–10.8 V VM operating range. If connected directly to VM, it drives the 12 V motors below their rated voltage, so full rated speed/performance should not be expected. Do not apply 12 V to the DRV8833; its recommended operating maximum is 10.8 V.

### 6. Changing the gyro axis after startup does not recalibrate its offset

At startup, `gyroOffset` is averaged only for the initially selected `PITCH_GYRO_AXIS`. The web interface lets the user change `gaxis`, but that setter does not recalculate `gyroOffset`. If the pitch axis is changed after boot, the bias subtraction may use an offset measured from the old axis. Reboot after changing the axis, or implement axis-specific offsets/recalibration.

### 7. The integral clamp is not saturation-aware anti-windup

The stored integral is bounded to ±400. This is a useful clamp, but it does not detect output saturation and stop/back-calculate integration based on actuator limits. The documentation should call it integral clamping rather than implying a complete anti-windup design.

Also, the integral is updated regardless of whether `useI` is false. Switching I off does not stop accumulation; switching it back on can reintroduce accumulated integral state.

### 8. Diagnostic gyro display applies one offset to all axes

The telemetry JSON subtracts the single pitch-axis `gyroOffset` from X, Y and Z display values. Only the selected pitch axis was calibrated with that offset. The non-pitch axis values displayed in the dashboard are therefore not independently bias-corrected.

## Reported performance comment in the source

A code comment states that the reference gains (`Kp=10`, `Ki=0`, `Kd=0.8`) previously held the robot upright for 36.4 seconds, with 3.1° angle standard deviation and no motor saturation. The comment does not provide a test protocol, date/year, battery condition, or raw log. Treat this as a historical developer note until it is reproduced and documented with test conditions.

## Before calling the firmware release-ready

- [ ] Identify the exact ESP32 board and validate every GPIO assignment.
- [ ] Identify the DRV8833 breakout; verify the reported onboard `nSLEEP` pull-up by schematic or measurement, and document the board model. The SLP header is reported externally unconnected.
- [ ] Confirm that the reported 0.3 A stall current is per motor at 12 V; identify the exact DRV8833 IC package/breakout and verify its thermal capability. On the reported figure alone the driver is not obviously undersized. Do not apply 12 V to this driver.
- [ ] Add a safe IMU-failure response and test it.
- [ ] Secure or disable the open Wi-Fi control interface.
- [ ] Recalibrate if the pitch gyro axis changes, or implement per-axis offsets.
- [ ] Decide whether to improve integral handling and rename the option accordingly.
- [ ] Run repeatable balance tests and retain logs.
