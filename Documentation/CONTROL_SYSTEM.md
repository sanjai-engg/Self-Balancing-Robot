# Control System: MPU6050 Kalman Estimator and PID

This document describes the implementation in `Firmware/PID_Control_Lab/PID_Control_Lab.ino`, rather than a generic balancing-robot design.

## 1. Control objective and scope

The robot is treated as an unstable inverted pendulum. The firmware controls the body lean angle around a setpoint (initially 0°) by driving the two wheels.

The implementation is **one angle-control loop**. The source explicitly does not read encoders and does not implement steering, wheel-speed control, or position control. Therefore, upright balance does not imply that the robot remains at one floor position; slow rolling/drift is not corrected by this controller.

## 2. Signal flow

1. Read the MPU6050 accelerometer and gyroscope registers over I²C.
2. Select the configured pitch-axis gyro reading and convert it to °/s.
3. Calculate the accelerometer tilt angle using the selected axis pair.
4. Fuse accelerometer angle and gyro rate with a two-state Kalman filter that estimates angle and gyro bias.
5. Calculate PID terms from the angle error and gyro rate.
6. Limit the signed motor command and apply motor deadband compensation.
7. Generate PWM outputs for the two DRV8833 H-bridge channels.

## 3. MPU6050 configuration and calibration

The sketch configures the MPU6050 directly via registers:

- Address: `0x68`
- I²C pins: SDA GPIO 8, SCL GPIO 9; bus clock 400 kHz
- Digital low-pass filter register: `0x03` (source comment identifies 44 Hz bandwidth)
- Sample-rate divider: `4` (configured 200 Hz sensor output rate)
- Gyroscope range: ±500 °/s (`GYRO_CONFIG = 0x08`)
- Accelerometer range: ±4 g (`ACCEL_CONFIG = 0x08`)
- Gyro conversion: 65.5 LSB/(°/s)
- Accelerometer conversion used for telemetry: 8192 LSB/g

At startup, the sketch averages 400 gyro readings while the robot is stationary and stores the pitch-axis offset in RAM. The `Set upright zero` web action records the accelerometer-derived upright angle trim in ESP32 Preferences (non-volatile storage).

Default orientation settings:

- Pitch gyro axis: Y (`PITCH_GYRO_AXIS = 1`)
- Gyro sign: −1
- Accelerometer pair: X/Z (`ACCEL_PAIR = 0`)
- Accelerometer sign: +1

These values must match the actual MPU6050 mounting orientation. The dashboard permits changing the axis and signs, but note the review item in [Firmware Review](FIRMWARE_REVIEW.md): changing the gyro axis after startup does not recalculate the startup offset.

## 4. Kalman angle and bias estimator

The function `fuseAngle(accelAngle, rate, dt)` implements a two-state Kalman estimator. Its estimated state is the tilt angle and the gyroscope bias:

\[
\mathbf{x} = \begin{bmatrix}\theta \\ b\end{bmatrix}
\]

The prediction step integrates the bias-corrected gyro rate:

\[
\theta^-_k = \theta_{k-1} + \Delta t(\omega_k-b_{k-1})
\]

The accelerometer-derived angle is then used as a measurement update. The filter calculates a two-component Kalman gain, corrects the angle estimate, and updates the estimated gyro bias.

The covariance/noise constants in the function are:

| Parameter | Value |
|---|---:|
| `Q_angle` | 0.001 |
| `Q_bias` | 0.003 |
| `R_measure` | 0.03 |

These are implementation constants in the supplied sketch. They should not be described as experimentally optimal unless a separate tuning/validation procedure supports that claim.

## 5. PID controller

The setpoint is an angle in degrees. The code calculates:

\[
e(t)=\text{setpoint}-\theta(t)
\]

and forms the three terms:

\[
P=K_p e(t), \qquad I=K_i\int e(t)\,dt
\]

By default, derivative-on-measurement is enabled. Instead of differentiating the error numerically, it uses the negative gyro rate:

\[
D=K_d(-\omega(t))
\]

With a fixed setpoint, this corresponds to the derivative of the error and avoids derivative kick when the setpoint changes. The code can switch to the numerical derivative `(error - prevError) / dt` through the dashboard or serial configuration.

The selected terms are summed:

\[
u=P+I+D
\]

The motor command is the negated, saturated controller output:

\[
\text{motorCmd}=\operatorname{clip}(-u,-255,255)
\]

The sign inversion is part of the firmware's chosen convention: a positive measured forward lean must result in wheel drive that moves the base forward under the body. The left and right motor outputs then receive opposite sign multipliers (`+1` and `−1`) to account for their mounting/wiring orientation.

### PID initial values and reference tune

| Parameter | Startup value | Reference-gain handler |
|---|---:|---:|
| `Kp` | 0 | 10.0 |
| `Ki` | 0 | 0.0 |
| `Kd` | 0 | 0.8 |
| Setpoint | 0° | 0° |

The gains start at zero deliberately. The dashboard's **Load reference gains** request sets `Kp=10`, `Ki=0`, `Kd=0.8`. This tune should be revalidated on the exact physical robot; differences in motors, battery, mass distribution, friction and wiring can make it unsuitable.

### Integral clamping

The code accumulates `error * dt` and, when `useAntiWindup` is enabled, clamps the stored integral state to ±400. This is an integral clamp. It limits growth but is not a saturation-aware back-calculation or conditional-integration scheme. The integral state is accumulated even when the I-term display/use checkbox is switched off; it can therefore have accumulated state when I is switched back on.

### Derivative implementation

`dOnMeasurement` is true by default, so `D = Kd * (-gyroRate)`. The alternative option differentiates error over the fixed `dt`. The source dashboard exposes both modes for comparison.

## 6. Timing and execution

- Nominal control frequency: 100 Hz
- Fixed control period: `DT = 0.01 s`
- Control task: FreeRTOS task pinned to core 1
- Task delay: `vTaskDelayUntil` with a nominal 10 ms period
- Measured loop frequency: reported by the dashboard as `loopHz`
- Serial telemetry: emitted every 20 ms (nominal 50 Hz)

The PID and Kalman updates use the fixed `DT` constant rather than calculating a fresh elapsed time for each update. Actual timing should be checked from the reported loop frequency and serial logs.

## 7. Motor actuation

- Driver identified by source comments: DRV8833
- Four PWM pins: GPIO 4, 5, 6, 7
- PWM frequency at startup: 20 kHz
- PWM resolution: 8 bits
- Output range: −255 to +255
- Default deadband: 30 counts
- Deadband ramp window: 10 counts
- Left motor sign: +1; right motor sign: −1

The firmware raises the command magnitude by a ramped deadband offset to help overcome static friction. This value depends on the actual motors, gearbox, battery voltage and mechanical load; it should be measured on the assembled robot.

## 8. Safety and fall detection

The controller disarms when the absolute estimated tilt exceeds 40°. When it disarms due to this threshold, the code clears the integral and stops driving on the following control-path branch.

**Important unresolved fault behaviour:** If `readIMU()` fails, the control task currently increments the error counter and immediately continues. It does not explicitly stop the motors or disarm first. This creates a risk that the last PWM command remains applied during an I²C failure. See [Firmware Review](FIRMWARE_REVIEW.md); correct and test this before considering the firmware fail-safe.

## 9. Dashboard, Wi-Fi, and serial interface

The ESP32 starts a Wi-Fi soft access point:

- SSID: `PID-Lab`
- Password: empty (open network)
- Dashboard: `http://192.168.4.1`

The web dashboard provides sensor telemetry, start/stop/zero controls, motor deadband test, gain sliders, P/I/D switches, derivative and integral options, step response test, and Ziegler–Nichols measurement helpers.

The sketch also accepts serial commands over USB at 115200 baud. Examples in the source include `P`, `I`, `D`, `GO`, `HALT`, `ZERO`, `DB`, `TEST`, `GAXIS`, `GSIGN`, `APAIR`, `ASIGN`, `SP`, `PWM`, and `RAMP`.

The dashboard has no authentication. Treat the open access point and unauthenticated motor-control endpoints as a safety/security issue; do not expose them to an uncontrolled network.

## 10. Architecture limitations

- No wheel encoders are read.
- No wheel-speed or position loop is implemented.
- No steering/yaw controller is implemented.
- The robot can balance its body angle while drifting in position.
- PID gains are not persisted across restart in the shown code; startup values are zero unless a reference tune or tuning input is applied during the session.
- The exact control quality and safe operating envelope require tests on the actual robot.
