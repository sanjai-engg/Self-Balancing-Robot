# Local Wi-Fi Tuning Dashboard

## Overview

The tuning dashboard is served directly by the ESP32-S3. It is not a separate cloud website and does not require an internet connection, a laptop acting as a server, or a third-party hosting service.

The HTML, CSS, and JavaScript for the dashboard are embedded in the firmware sketch in the `PAGE[] PROGMEM` string. The ESP32 serves that page using the Arduino `WebServer` on HTTP port 80.

Source implementation: [`Firmware/PID_Control_Lab/PID_Control_Lab.ino`](../Firmware/PID_Control_Lab/PID_Control_Lab.ino).

## Connect to the dashboard

1. Power the robot and allow the ESP32 to start.
2. On a phone, tablet, or laptop, open the Wi-Fi network list.
3. Connect to the ESP32 access point:
   - **SSID:** `PID-Lab`
   - **Password:** none configured in the current sketch (open network)
4. Keep the device connected to this Wi-Fi network. It may report that the network has no internet access; that is expected.
5. Open a browser and visit **http://192.168.4.1**.

The phone or computer communicates directly with the ESP32 over the local Wi-Fi network. Internet access is not needed.

## Dashboard capabilities in the current firmware

The dashboard currently provides:

- **Live status and telemetry:** balancing state, estimated pitch angle, gyro rate, PID terms, motor command, sensor readings, measured loop frequency, and IMU error count.
- **Start/stop and zeroing:** arm balancing, stop the controller, and set the upright angle reference.
- **Sensor-orientation checks:** view raw gyro and accelerometer values; select the pitch gyro axis, gyro sign, accelerometer axis pair, and accelerometer sign.
- **Motor deadband test:** command the motors while balancing is stopped and adjust the deadband setting.
- **PID tuning:** change (K_p), (K_i), and (K_d) while the firmware is running; enable or disable the P, I, and D contributions.
- **Controller options:** switch between derivative-on-measurement and derivative-on-error, and enable or disable the integral clamp.
- **Response testing:** apply a 3-degree setpoint step and view the measured response values, including overshoot, rise time, settling time, and steady-state error.
- **Oscillation / Ziegler–Nichols helpers:** view measured oscillation period and peak-to-peak swing, record (K_u) and (T_u), and apply the corresponding tuning calculation exposed by the sketch.
- **Reference gains:** load the code's reference values ((K_p=10.0), (K_i=0.0), (K_d=0.8)). These are a previously reported tune, not a guarantee of safe or optimal operation on every build.

## What is and is not saved

The current sketch stores the upright angle trim in ESP32 Preferences. The PID gains and most dashboard tuning changes are held in RAM and should be treated as temporary; they are not saved as a persistent user profile by the current implementation. After a reboot, the gains return to their startup values unless they are loaded or tuned again.

Changing the gyro axis from the dashboard after startup does not recalculate the startup gyro offset. Confirm the correct sensor orientation and signs carefully before relying on a tune.

## Safety and security

**The current access point is open and the dashboard has no authentication.** Any nearby person who joins the `PID-Lab` network can reach the motor-control and tuning endpoints. The dashboard includes commands that can start the motors; it is not a read-only monitor.

- Use the dashboard only in a controlled test area.
- Keep the wheels clear of people, loose wires, and objects during motor tests.
- Have a physical way to remove motor power and stop the robot.
- Do not expose this open access point or unauthenticated control interface to an uncontrolled environment.
- Before public demonstration or broader use, configure appropriate Wi-Fi security and add application-level protection for motor-control actions.

There is also a firmware fault-handling issue: if an IMU read fails, the current control task continues without first explicitly disarming and stopping the motors. This should be corrected and tested before treating the system as fail-safe.

## Current architecture and future improvements

The current implementation is a single-device local setup:

`Phone / Laptop browser  <— local Wi-Fi —>  ESP32 SoftAP + HTTP server  <—>  robot controller`

No router or internet connection is required. The dashboard is embedded in the sketch, so changing its HTML/CSS/JavaScript currently means editing the firmware and uploading it to the ESP32.

Potential future improvements include password-protected Wi-Fi, authenticated control requests, persistent PID profiles, exportable tuning logs, and separating the web assets from the sketch if the firmware is later changed to serve files from a filesystem such as LittleFS.
