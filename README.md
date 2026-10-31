# ESP32 Self-Balancing Robot

A two-wheeled, balancing robot using an **ESP32-S3-WROOM-1 module**, an **MPU6050 IMU**, a **DRV8833 dual H-bridge motor-driver breakout**, **two 12 V, 300 RPM N20 DC geared motors**, and a **2S LiPo battery**. The firmware uses a two-state Kalman filter to estimate pitch angle and gyro bias, followed by a PID angle controller. The mechanical design was created in **Autodesk Fusion**.

> **Documentation status:** This README has been checked against the supplied Arduino sketch and the wiring details reported by the project author. The reported battery-to-driver and motor-output connections are recorded below. The exact carrier/development-board model, exact DRV8833 breakout model/revision, the breakout's onboard `nSLEEP` pull-up implementation, the exact silkscreen spelling of the motor-supply terminal (reported as `ULT`, interpreted as `VM`/motor supply), N20 motor manufacturer/gear ratio, the basis of the reported 0.3 A stall current, battery capacity/discharge rating, and ESP32/MPU6050 power-regulation details still need verification.

## Project overview

The robot is modelled as an unstable inverted pendulum. The controller estimates pitch angle and commands the wheels to move the base under the body to oppose a fall.

The implemented firmware performs **angle balancing only**. It does not read wheel encoders and does not implement steering, wheel-speed regulation, or position control. Consequently, the robot may drift across the floor or bench while maintaining an approximately upright angle. This is an expected limitation of the implemented control architecture, not evidence of position holding.

## Hardware and wiring

| Component | Identified specification |
|---|---|
| Microcontroller | ESP32-S3-WROOM-1 module; exact carrier/development-board model TODO |
| IMU | MPU6050, I²C address `0x68` |
| Motor driver | DRV8833 dual H-bridge breakout; exact board/revision TODO |
| Motors | Two N20 DC geared motors, 12 V, 300 RPM (author-reported) |
| Motor stall current | 0.3 A reported; confirm this is per motor and specified at 12 V |
| Battery | 2S LiPo; battery positive reported connected directly to the DRV8833 motor-supply terminal; exact pack details TODO |
| CAD | Autodesk Fusion |

### ESP32 pin assignments from the firmware

| Function | ESP32 GPIO |
|---|---:|
| MPU6050 SDA | GPIO 8 |
| MPU6050 SCL | GPIO 9 |
| Left motor control inputs | GPIO 4 and GPIO 5 |
| Right motor control inputs | GPIO 6 and GPIO 7 |

The firmware uses GPIO4/5 for the left motor's two H-bridge inputs and GPIO6/7 for the right motor's two H-bridge inputs. The author reports that the left motor is connected to driver outputs `OUT1`/`OUT2`, and the right motor to `OUT3`/`OUT4`. On the usual DRV8833 channel naming, `OUT1`/`OUT2` are controlled by `AIN1`/`AIN2`, and `OUT3`/`OUT4` by `BIN1`/`BIN2`; therefore the expected input mapping is GPIO4/5 → AIN1/AIN2 and GPIO6/7 → BIN1/BIN2. Confirm this last input-wire mapping against the physical board before publishing a final schematic—the code comments alone do not prove the terminal-to-wire mapping.

### Battery, driver and motor connections reported by the author

- 2S LiPo positive is connected directly to the DRV8833 breakout motor-supply input. The pin was transcribed by the author as **`ULT`** and described as the motor-voltage / `VM` pin. **Check the silkscreen or a board photo:** `VM`, `VMotor`, or a similar label is common; `ULT` is not the standard DRV8833 IC pin name.
- 2S LiPo negative is connected to the DRV8833 `GND` terminal.
- The author describes this as part of a shared common-ground network. Confirm that ESP32 GND and MPU6050 GND are also tied to this same reference.
- Driver outputs `OUT1` and `OUT2` connect to the left N20 motor.
- Driver outputs `OUT3` and `OUT4` connect to the right N20 motor.
- The driver `SLP`/`nSLEEP` header is physically left without an external wire. The author believes the breakout has an onboard pull-up that enables the driver. This is a **reported board-level feature, not yet independently verified** because the breakout manufacturer/model and schematic have not been identified.

**Important nSLEEP distinction:** the DRV8833 IC itself has an internal 500 kΩ pull-down from `nSLEEP` to GND. A bare IC with `nSLEEP` left floating therefore defaults to sleep, not enabled. If this robot's driver works with no external `SLP` wire, the breakout must provide some board-level pull-up or there must be another electrical path. Do not describe this as an internal pull-up in the DRV8833 chip. Check the exact breakout schematic or measure `SLP` relative to driver GND with a high-impedance multimeter (motors disconnected or robot securely restrained). TI specifies `nSLEEP` logic-high at 2.5 V minimum and describes the internal clamp at 6.5 V; if pulling up to `VM`, TI recommends a 20 kΩ–75 kΩ resistor. Do not connect `SLP` directly to the 2S battery positive without verifying the board's circuit.

### Voltage and current compatibility

A conventional 2S LiPo is 7.4 V nominal and 8.4 V when fully charged (verify the exact battery chemistry and label). TI specifies a 2.7–10.8 V operating range for the DRV8833 motor supply, so a standard 2S pack connected directly to `VM` is within the IC's specified voltage range. The author's reported 12 V N20 motors are consequently being supplied below their rated voltage; reduced speed and available performance should be expected compared with 12 V operation. Do **not** raise the DRV8833 motor-supply rail to 12 V: that exceeds its 10.8 V recommended operating maximum.

The author reports 0.3 A stall current. The current basis is not yet fully specified: confirm whether this is **per motor at 12 V** and is the manufacturer's stall-current value. If so, it is below TI's lowest stated package-specific continuous RMS/DC current figure of 0.5 A per H-bridge for the PW/TSSOP package option. This makes the driver not obviously undersized on the reported current alone, but is not a final thermal sign-off: exact IC package, breakout PCB copper/thermal design, both channels operating, motor stall duration and measured board temperature matter. TI's PWP/RTY package options have higher ratings; consult the exact package and carrier documentation rather than treating peak-current figures as continuous ratings.

For the detailed wiring status, electrical checks and cautions, see [Hardware and Wiring](Documentation/HARDWARE_AND_WIRING.md), [Power Compatibility](Documentation/POWER_COMPATIBILITY.md), and [Verified Pinout](Hardware/PINOUT.md).

## Software and control architecture

- **IDE / language:** Arduino IDE / C++ (Arduino framework)
- **Sensor acquisition:** direct MPU6050 register access over I²C using `Wire`; no external MPU6050 library is included in the sketch
- **Angle estimation:** two-state Kalman filter estimating angle and gyro bias
- **Controller:** single PID loop; derivative-on-measurement is enabled by default and uses gyro rate
- **Control task:** pinned to ESP32 core 1, nominal 100 Hz; fixed `DT = 0.01 s`
- **Motor output:** four LEDC PWM outputs, 20 kHz and 8-bit resolution
- **Monitoring/tuning:** Wi-Fi soft access point, browser dashboard, and USB serial command/telemetry interface

See [Control System](Documentation/CONTROL_SYSTEM.md) for implementation details extracted from the sketch.

## Repository layout

```text
Self-Balancing-Robot/
├── README.md
├── .gitignore
├── Firmware/
│   └── PID_Control_Lab/
│       └── PID_Control_Lab.ino
├── Hardware/
│   ├── PINOUT.md
│   └── BILL_OF_MATERIALS.md
├── CAD/
│   ├── Source-Files/       # Native Autodesk Fusion files
│   └── Exports/            # STEP/STL and other exports
├── Media/
│   └── Photos/
└── Documentation/
    ├── HARDWARE_AND_WIRING.md
    ├── POWER_COMPATIBILITY.md
    ├── CONTROL_SYSTEM.md
    ├── FIRMWARE_REVIEW.md
    ├── CAD_AND_ASSEMBLY.md
    ├── TESTING_AND_RESULTS.md
    └── UPLOAD_CHECKLIST.md
```

## Mechanical design

The mechanical design was created in **Autodesk Fusion**. Add the editable Fusion project/design files to `CAD/Source-Files/` and suitable STEP/STL exports to `CAD/Exports/`. Add assembly photographs to `Media/Photos/`. The current information does not establish the chassis material/dimensions, wheel dimensions, gear ratio, motor manufacturer, or fastener specifications.

See [CAD and Assembly](Documentation/CAD_AND_ASSEMBLY.md).

## Demonstration

**TODO:** Add a link to a demonstration video. A hosted video link is generally preferable to committing a large video file directly to Git.

## Results

The source contains a comment reporting one run with the reference tune (`Kp = 10.0`, `Ki = 0.0`, `Kd = 0.8`): 36.4 seconds upright, 3.1° angle standard deviation, and no motor saturation. Treat this as a **source-code comment about a reported run**, not as an independently verified or repeatable performance result. Re-run the test on the current physical build and record the conditions in [Testing and Results](Documentation/TESTING_AND_RESULTS.md) before using it as a project claim.

## Safety and known limitations

- The Wi-Fi access point is named `PID-Lab` and is configured with an empty password. The HTTP dashboard has no authentication. Anyone able to join that network can issue motor-control and tuning requests. Do not expose this control interface in an uncontrolled environment; add authentication or disable the AP before public/deployed use.
- In the current sketch, an IMU read failure increments an error counter and immediately continues the control task. It does not first disarm the controller or explicitly stop the motors. This should be corrected and tested before treating the firmware as fail-safe.
- The DRV8833 chip has an internal `nSLEEP` pull-down, not a pull-up. The author reports an onboard breakout pull-up while leaving the SLP header without an external wire; verify the module schematic or measure the pin. Do not tie SLP directly to the 2S battery without an appropriate resistor/network.
- The ESP32-S3-WROOM-1 module-level GPIO assignment is compatible, but verify the exact carrier-board header routing and onboard peripherals.
- The motor supply is reported connected directly to the 2S pack. This is within the DRV8833 supply range but below the N20 motors' 12 V rating. Do not apply 12 V to the DRV8833.
- The reported 0.3 A stall current still needs confirmation as per-motor and at 12 V; identify the exact DRV8833 breakout/package and verify thermal behaviour.
- The firmware has no encoder feedback, steering loop, wheel-speed loop, or position loop.
- Initial PID gains are zero. The robot will not balance until suitable gains are loaded/tuned. The dashboard's reference-gain button loads one previously reported tune; it is not a guarantee that the tune is suitable for every build.

See [Firmware Review](Documentation/FIRMWARE_REVIEW.md) and [Hardware and Wiring](Documentation/HARDWARE_AND_WIRING.md).

## License

**TODO:** Choose a license deliberately. Firmware, CAD files, photos, and documentation may have different reuse terms. Until a license is added, do not imply that reuse or redistribution is permitted.

## Author

**TODO:** Add author name, project date, and optional profile/contact links.
