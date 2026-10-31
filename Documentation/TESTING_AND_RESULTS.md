# Testing and Results

> Record measured observations only. A code comment about an earlier run is not a substitute for repeatable results from the documented build.

## 1. Test setup

| Item | Test condition |
|---|---|
| Test date | TODO |
| Exact ESP32 board | TODO |
| Surface | TODO: material and condition |
| Battery type / voltage at start | TODO |
| Robot configuration / payload | TODO |
| Firmware Git commit | TODO |
| MPU6050 orientation / calibration | TODO |
| Kalman parameters | `Q_angle=0.001`, `Q_bias=0.003`, `R_measure=0.03` (unless changed) |
| PID gains | TODO: record actual `Kp`, `Ki`, `Kd` |
| Setpoint | TODO; firmware initial value is 0° |
| Control-loop frequency | Nominal 100 Hz; record observed `loopHz` |

## 2. Functional checks

| Test | Procedure | Result | Notes |
|---|---|---|---|
| IMU communication | Confirm MPU6050 reads succeed and error count remains stable | TODO | I²C address `0x68` in code |
| Pitch axis and sign | Tilt robot forward by hand; confirm angle and gyro rate signs match the convention | TODO | Default gyro Y, sign −1; accel X/Z, sign +1 |
| Gyro calibration | Hold the robot stationary through startup calibration | TODO | Firmware averages 400 selected-axis readings |
| Upright trim | Hold robot vertical and use `Set upright zero` | TODO | Trim stored in ESP32 Preferences |
| Motor direction | Test each motor at low command with wheels clear of floor | TODO | Positive command must move base forward to catch forward lean |
| Motor deadband | Use motor test to find the smallest command that starts motion | TODO | Default deadband 30, ramp window 10 |
| IMU failure response | Interrupt sensor communications under a safe restrained test | TODO | Correct fail-safe behaviour before release |
| Wi-Fi access control | Verify AP password and endpoint access restrictions | TODO | Current supplied code uses open AP and no authentication |
| Balance test | Test on a level surface with safe restraint and power isolation available | TODO | State trial duration and success criterion |

## 3. Quantitative results

| Metric | Measured value | Conditions |
|---|---:|---|
| Observed control-loop frequency | TODO Hz | Record dashboard `loopHz` |
| Continuous balancing duration | TODO s/min | State number of trials and battery voltage |
| Angle standard deviation | TODO ° | Define measurement window and logging source |
| Maximum tilt / recovery range | TODO ° | Define disturbance protocol |
| Motor command saturation | TODO | State whether command reached ±255 |
| Battery operating time | TODO min | Battery model and load |
| Battery voltage before/after test | TODO V | State measurement point |

### Historical note from the supplied source

The sketch contains a comment reporting a 36.4-second run with 3.1° angle standard deviation and no motor saturation at reference gains `Kp=10.0`, `Ki=0.0`, `Kd=0.8`. The source comment does not state a complete test protocol. Reproduce the result and record conditions before presenting it as a verified project result.

## 4. Repeatable test protocol

Record the starting angle, battery state, surface, firmware commit, tuning values, restraint/recovery method, run duration and pass/fail criteria. Report the number of trials, not only the best run.

## 5. Observed issues and corrective actions

| Observation | Cause investigated | Change made | Outcome |
|---|---|---|---|
| TODO | TODO | TODO | TODO |

## 6. Known limitations

The supplied firmware regulates body angle only; it does not read encoders or control wheel speed/position. Record any additional limits observed on your physical robot, such as uneven surfaces, low battery, payload sensitivity, sensor vibration, motor deadband or sustained drift.
