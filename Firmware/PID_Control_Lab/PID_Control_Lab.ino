/* ===========================================================================
 * PID CONTROL LAB - Two-wheel self-balancing robot
 * ===========================================================================
 *
 * A teaching rig. The controller is deliberately the plainest possible
 * textbook PID so that every term is visible and can be switched off, and the
 * web page reports the numbers you need to fill in a lab report.
 *
 * ---------------------------------------------------------------------------
 * THE PROBLEM
 * ---------------------------------------------------------------------------
 * The robot is an inverted pendulum: balanced upright it is in EQUILIBRIUM,
 * but an UNSTABLE one. Nudge it and gravity pulls it further from upright, not
 * back towards it. Left alone it always falls.
 *
 * The only thing we can do about it is drive the wheels. Drive them forward
 * and the base moves under the body, which rotates the body backwards. So:
 *
 *      to stop the robot falling FORWARD, drive the wheels FORWARD.
 *
 * That is the whole physical idea. Everything below is about deciding HOW
 * HARD to drive them.
 *
 * ---------------------------------------------------------------------------
 * SCOPE: BALANCING ONLY
 * ---------------------------------------------------------------------------
 * There is no steering, no speed command and no position control here. One
 * PID loop, one job: hold the lean angle at the setpoint. The encoders are not
 * even read.
 *
 * A consequence worth discussing in the lab: the robot will slowly drift along
 * the bench even when it is balancing perfectly. That is not a fault and no
 * amount of PID tuning will remove it. The controller is being asked to
 * regulate ANGLE, and it does - but "upright" and "stationary" are different
 * goals, and nothing in this loop measures position. Holding position needs a
 * second, outer loop that watches the wheels and asks this loop for a slightly
 * different angle. That is deliberately out of scope.
 *
 * ---------------------------------------------------------------------------
 * THE CONTROLLER
 * ---------------------------------------------------------------------------
 * We measure the lean angle and compare it to where we want it:
 *
 *      error = setpoint - angle          (degrees)
 *
 * and compute a motor command from three separate terms:
 *
 *      P term = Kp * error               "how far off am I RIGHT NOW?"
 *      I term = Ki * integral(error dt)   "how long have I been off?"
 *      D term = Kd * d(error)/dt          "how fast is the error changing?"
 *
 *      output = P + I + D
 *
 * Each term answers a different question about the error, and each has a
 * characteristic failure mode when it is too large:
 *
 *      too much P  ->  oscillation (it overcorrects, every time)
 *      too much I  ->  slow, growing wobble (it keeps pushing after arriving)
 *      too much D  ->  buzzing, twitching (it amplifies sensor noise)
 *
 * Every one of these can be produced on purpose with this rig. Do so - the
 * failure modes are more instructive than the working tune.
 *
 * ---------------------------------------------------------------------------
 * HARDWARE
 * ---------------------------------------------------------------------------
 *   ESP32 (Arduino core 3.x)
 *   MPU-6050 IMU     I2C on SDA=8, SCL=9
 *   DRV8833 driver   IN1..IN4 on 4, 5, 6, 7
 *
 * Connect to the Wi-Fi network "PID-Lab" and open http://192.168.4.1
 * =========================================================================== */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ===========================================================================
// SECTION 1 - Hardware configuration
// ===========================================================================
const int  I2C_SDA = 8, I2C_SCL = 9;
const int  MPU_ADDR = 0x68;

const int  PWM_LA = 4, PWM_LB = 5;      // left motor
const int  PWM_RA = 6, PWM_RB = 7;      // right motor

// Which gyro axis is the pitch axis, and which way round.
// CHECK THIS FIRST: tip the robot forward and watch the web page. "angle" and
// "gyro" must BOTH go positive. If gyro hardly moves you have the wrong axis;
// if it moves the wrong way, flip GYRO_SIGN.
// MEASURED on this robot, 9 Sep: tilting it gave 12x more motion on gyro Y
// than on X or Z, so Y is the pitch axis; and the correlation between gyro Y
// and d(accel angle)/dt came out NEGATIVE (-0.59), so the gyro reads backwards
// relative to the accelerometer. Replaying the captured motion through this
// sketch's own Kalman filter: with +1 the fused angle lagged reality by 600 ms
// and under-read by up to 49 deg; with -1 it tracks with 0 ms lag and 0.46 deg
// mean error. That 600 ms was the "delayed, insensitive" behaviour.
int   PITCH_GYRO_AXIS = 1;              // 0 = X, 1 = Y, 2 = Z
float GYRO_SIGN       = -1.0;
float ACCEL_SIGN      = 1.0;
int   ACCEL_PAIR      = 0;              // 0 = atan2(ax,az), 1 = atan2(ay,az), 2 = atan2(ax,ay)

// All four are settable from the web page, so the correct combination can be
// found by watching the numbers rather than by editing and re-uploading.

const float GYRO_LSB  = 65.5;           // LSB per deg/s at +/-500 deg/s

// A positive motor command must drive the robot FORWARD.
const float MOTOR_L_SIGN = 1.0;
const float MOTOR_R_SIGN = -1.0;

const int   LOOP_HZ = 100;              // control loop rate
const float DT      = 1.0 / LOOP_HZ;

// ===========================================================================
// SECTION 2 - The PID gains, and the switches that make this a lab
// ===========================================================================
// The gains start at ZERO on purpose. A robot that balances out of the box
// teaches nothing. You are going to find these values yourself.
float Kp = 0.0;      // PWM counts per degree of error
float Ki = 0.0;      // PWM counts per (degree * second)
float Kd = 0.0;      // PWM counts per (degree / second)

float setpoint = 0.0;   // the angle we are asking for, in degrees

// Each term can be switched out to see what it was contributing.
bool useP = true, useI = true, useD = true;

// Two implementation choices that textbooks gloss over but that matter a great
// deal in practice. Both are explained where they are used, in Section 6.
bool dOnMeasurement = true;   // false = textbook d(error)/dt
bool useAntiWindup  = true;   // false = let the integral wind up freely

// ===========================================================================
// SECTION 3 - State
// ===========================================================================
Preferences prefs;
WebServer   server(80);

int16_t accelX, accelY, accelZ, gyroRaw[3];
float   gyroOffset = 0;
float   angleTrim  = 0;          // accelerometer reading when truly upright

float angle = 0, gyroRate = 0;   // fused angle [deg], pitch rate [deg/s]
float kBias = 0;                 // Kalman gyro-bias state
float kP[2][2] = {{1,0},{0,1}};

float integral = 0, prevError = 0;
float pTerm = 0, iTerm = 0, dTerm = 0, output = 0;
int   motorCmd = 0;

bool  balancing = false;         // armed / disarmed
unsigned long imuErrors = 0;
volatile float loopHz = 0;       // measured, not assumed

// --- measurements the lab needs -------------------------------------------
float  oscPeriod = 0;            // measured oscillation period Tu [s]
float  oscAmplitude = 0;         // peak-to-peak swing [deg]
float  Ku_recorded = 0, Tu_recorded = 0;

bool   stepRunning = false;
unsigned long stepStartMs = 0;
float  stepFrom = 0, stepTo = 0;
float  stepPeak = 0, stepRise = 0, stepSettle = 0, stepSS = 0;
bool   stepDone = false;

// trace for the response plot (2 Hz decimation of a 100 Hz loop => 12 s)
const int TRACE_N = 120;
float traceAngle[TRACE_N], traceSet[TRACE_N];
int   traceHead = 0, traceDiv = 0;

bool readIMU();
void controlLoop(void *);

// ===========================================================================
// SECTION 4 - Sensors
// ===========================================================================
bool readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) { imuErrors++; return false; }
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) { imuErrors++; return false; }

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                        // temperature
  for (int i = 0; i < 3; i++) gyroRaw[i] = Wire.read() << 8 | Wire.read();
  return true;
}

/* Sensor fusion, in one paragraph.
 *
 * The accelerometer measures which way gravity points, so it gives an absolute
 * angle - but it is noisy and it cannot tell gravity apart from the robot's own
 * acceleration. The gyro measures rotation RATE very cleanly, but to get an
 * angle you must integrate it, and any small offset integrates into a drift
 * that grows without limit.
 *
 * So: trust the gyro over the short term, and use the accelerometer to slowly
 * pull the answer back to the truth. This is a Kalman filter, which does
 * exactly that and also estimates the gyro's offset as it goes.
 *
 * This is NOT the subject of the lab - treat it as a black box that gives you
 * a good angle. The lab is about what you do with that angle.
 */
float fuseAngle(float accelAngle, float rate, float dt) {
  const float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

  angle += dt * (rate - kBias);                      // predict using the gyro
  kP[0][0] += dt * (dt*kP[1][1] - kP[0][1] - kP[1][0] + Q_angle);
  kP[0][1] -= dt * kP[1][1];
  kP[1][0] -= dt * kP[1][1];
  kP[1][1] += Q_bias * dt;

  float S = kP[0][0] + R_measure;                    // correct using the accel
  float K0 = kP[0][0]/S, K1 = kP[1][0]/S;
  float y  = accelAngle - angle;
  angle += K0 * y;
  kBias += K1 * y;

  float p00 = kP[0][0], p01 = kP[0][1];
  kP[0][0] -= K0*p00;  kP[0][1] -= K0*p01;
  kP[1][0] -= K1*p00;  kP[1][1] -= K1*p01;
  return angle;
}

// ===========================================================================
// SECTION 5 - Motors
// ===========================================================================
// Static friction means the wheels do not move at all until the command passes
// some threshold. Right at the balance point, where we need the finest
// control, we would otherwise have none. So we add the threshold back in.
// Adjustable, because the real value depends on your motors, gearbox and
// battery state. If it is set too low, small corrections produce no wheel
// movement at all and the robot feels sluggish and insensitive no matter what
// you do to the gains. Find it with the motor test on the web page.
int DEADBAND = 30;
int motorTest = 0;          // open-loop drive while stopped, for finding it
int RAMPWIN   = 10;         // counts over which the deadband is faded in
int PWMFREQ   = 20000;      // Hz

// Why PWM frequency matters here: a brushed motor is an inductor, so at high
// switching frequency the current barely ripples and, at low duty, never rises
// enough to break static friction. The wheels then do nothing at all until the
// command is fairly large - a wide dead zone exactly around the balance point,
// where the finest control is needed. Dropping the frequency lets the current
// ripple, which breaks stiction and narrows that dead zone. The cost is
// audible whine.

void writeMotor(int pinA, int pinB, int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (cmd >= 0) { ledcWrite(pinB, 0); ledcWrite(pinA, cmd); }
  else          { ledcWrite(pinA, 0); ledcWrite(pinB, -cmd); }
}

void drive(int cmd) {
  if (cmd != 0) {
    // Ramp the deadband in over the first few counts rather than jumping
    // straight to +/-30, which would make the wheels chatter across zero.
    float ramp = min(fabs((float)cmd) / (float)max(RAMPWIN, 1), 1.0f);
    cmd += (cmd > 0 ? 1 : -1) * (int)(DEADBAND * ramp);
    cmd = constrain(cmd, -255, 255);
  }
  writeMotor(PWM_LA, PWM_LB, (int)(MOTOR_L_SIGN * cmd));
  writeMotor(PWM_RA, PWM_RB, (int)(MOTOR_R_SIGN * cmd));
}

void stopMotors() {
  ledcWrite(PWM_LA, 0); ledcWrite(PWM_LB, 0);
  ledcWrite(PWM_RA, 0); ledcWrite(PWM_RB, 0);
}

// ===========================================================================
// SECTION 6 - THE PID CONTROLLER
// ===========================================================================
// This is the part the lab is about. Read it line by line.
// ===========================================================================
void runPID(float dt) {

  // ---- The error. Everything below is a different opinion about this one
  // ---- number: its size, its history, and its rate of change.
  float error = setpoint - angle;

  // ---- PROPORTIONAL ------------------------------------------------------
  // Push back in proportion to how wrong we are. This is the main effort.
  //
  // On its own it is never quite enough: to hold a steady push (a slope, a
  // battery on one side) P needs a steady error to work on. It therefore
  // settles at the WRONG angle. That leftover is "steady-state error", and
  // removing it is exactly the I term's job.
  pTerm = Kp * error;

  // ---- INTEGRAL ----------------------------------------------------------
  // Accumulate the error over time. As long as any error remains, this keeps
  // growing, so the push keeps rising until the error is actually zero. That
  // is how steady-state error is eliminated.
  //
  // The catch: if the motors are already saturated the error cannot shrink, so
  // the integral keeps growing into a number the robot can never "spend". When
  // things finally do respond, all that stored-up push has to be unwound - a
  // huge overshoot. This is INTEGRAL WINDUP, and clamping is the usual cure.
  // Switch useAntiWindup off and lay the robot on its side to see it happen.
  integral += error * dt;
  if (useAntiWindup) integral = constrain(integral, -400, 400);
  iTerm = Ki * integral;

  // ---- DERIVATIVE --------------------------------------------------------
  // React to how fast the error is changing, not how big it is. It applies the
  // brakes as we approach the target, which damps the oscillation that P
  // causes on its own.
  //
  // Two ways to compute it, and the difference matters:
  //
  //   dOnMeasurement = false   the textbook form, d(error)/dt. Dividing a
  //                            difference by dt multiplies sensor noise by
  //                            1/dt - at 100 Hz that is a gain of 100 on the
  //                            noise. It also spikes hard whenever you change
  //                            the setpoint, because the error jumps
  //                            instantly. That spike is "derivative kick".
  //
  //   dOnMeasurement = true    use the gyro, which measures the rate of change
  //                            of the angle directly, with no differentiation
  //                            at all. With a fixed setpoint the two are the
  //                            same quantity: d(error)/dt = -d(angle)/dt.
  //
  // Run the step test in both modes and compare. This is one of the clearest
  // demonstrations in the lab.
  float derivative;
  if (dOnMeasurement) derivative = -gyroRate;
  else                derivative = (error - prevError) / dt;
  prevError = error;
  dTerm = Kd * derivative;

  // ---- SUM ---------------------------------------------------------------
  output = 0;
  if (useP) output += pTerm;
  if (useI) output += iTerm;
  if (useD) output += dTerm;

  // Leaning forward is a positive angle, and the wheels must go forward to
  // catch it - the opposite sign to the restoring effort computed above.
  motorCmd = constrain((int)(-output), -255, 255);
}

// ===========================================================================
// SECTION 7 - Lab measurements
// ===========================================================================
// Oscillation period, by timing successive upward crossings of the setpoint.
// Needed for the Ziegler-Nichols method in Exercise 4.
void measureOscillation(float dt) {
  static float prevA = 0;
  static unsigned long lastCross = 0;
  static float hi = 0, lo = 0;
  static unsigned long windowStart = 0;

  unsigned long now = millis();
  if (angle > hi) hi = angle;
  if (angle < lo) lo = angle;

  if (now - windowStart > 2000) {          // report peak-to-peak every 2 s
    oscAmplitude = hi - lo;
    hi = lo = angle;
    windowStart = now;
  }

  if (prevA < setpoint && angle >= setpoint) {   // rising crossing
    if (lastCross != 0) {
      float period = (now - lastCross) / 1000.0;
      if (period > 0.05 && period < 5.0) {
        oscPeriod = 0.7 * oscPeriod + 0.3 * period;   // smooth it a little
      }
    }
    lastCross = now;
  }
  prevA = angle;
}

// A step in the setpoint, and the four numbers that describe the response.
void runStepTest() {
  if (!stepRunning) return;

  float t = (millis() - stepStartMs) / 1000.0;
  float span = stepTo - stepFrom;

  if (span > 0 ? (angle > stepPeak) : (angle < stepPeak)) stepPeak = angle;

  // Rise time: first arrival at the commanded value.
  if (stepRise == 0 && ((span > 0 && angle >= stepTo) || (span < 0 && angle <= stepTo))) {
    stepRise = t;
  }
  // Settling time: last moment it was still outside a +/-0.5 deg band.
  if (fabs(angle - stepTo) > 0.5) stepSettle = t;

  if (t > 3.0) {   // kept short: with no position loop the robot rolls during this
    stepRunning = false;
    stepDone = true;
    stepSS = angle - stepTo;               // steady-state error
    setpoint = stepFrom;
  }
}

void startStepTest() {
  if (!balancing) return;
  stepFrom = setpoint;
  stepTo   = setpoint + 3.0;               // a 3 degree step
  stepPeak = angle;
  stepRise = stepSettle = stepSS = 0;
  stepDone = false;
  stepStartMs = millis();
  stepRunning = true;
  setpoint = stepTo;
}

// ===========================================================================
// SECTION 8 - Control task (core 1)
// ===========================================================================
float accelPitch() {
  float n, d;
  switch (ACCEL_PAIR) {
    case 1:  n = accelY; d = accelZ; break;
    case 2:  n = accelX; d = accelY; break;
    default: n = accelX; d = accelZ; break;
  }
  return ACCEL_SIGN * atan2(n, d) * 180.0 / PI;
}

void controlLoop(void *) {
  TickType_t wake = xTaskGetTickCount();
  unsigned long tick = 0, t0 = millis();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / LOOP_HZ));

    if (++tick >= 100) { loopHz = tick * 1000.0 / (millis() - t0); t0 = millis(); tick = 0; }

    if (!readIMU()) continue;

    gyroRate = GYRO_SIGN * (gyroRaw[PITCH_GYRO_AXIS] - gyroOffset) / GYRO_LSB;
    float accelAngle = accelPitch() - angleTrim;
    angle = fuseAngle(accelAngle, gyroRate, DT);

    measureOscillation(DT);
    runStepTest();

    // Fall over -> stop, and clear the integral so it does not unleash
    // everything it stored while lying on its side.
    if (fabs(angle) > 40.0) {
      balancing = false;
      integral = 0;
      stepRunning = false;
    }

    if (balancing) {
      runPID(DT);
      drive(motorCmd);
    } else if (motorTest != 0) {
      motorCmd = motorTest;          // open-loop, for finding the deadband
      drive(motorCmd);
    } else {
      motorCmd = 0;
      integral = 0;
      stopMotors();
    }

    if (++traceDiv >= 4) {                 // 25 Hz into the plot
      traceDiv = 0;
      traceAngle[traceHead] = angle;
      traceSet[traceHead]   = setpoint;
      traceHead = (traceHead + 1) % TRACE_N;
    }
  }
}

// ===========================================================================
// SECTION 9 - Web interface
// ===========================================================================
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PID Lab</title><style>
body{margin:0;padding:14px;font:14px system-ui,sans-serif;background:#0f1318;color:#dfe6ee;
     max-width:620px;margin:0 auto}
h1{font-size:17px;margin:0 0 4px}
h2{font-size:11px;text-transform:uppercase;letter-spacing:.09em;color:#7d8b9a;
   margin:20px 0 8px;border-bottom:1px solid #232b34;padding-bottom:4px}
.card{background:#161c23;border:1px solid #232b34;border-radius:8px;padding:12px;margin-bottom:10px}
.st{display:inline-block;padding:4px 10px;border-radius:4px;font-weight:700;font-size:12px}
.mono{font-family:ui-monospace,Menlo,monospace}
canvas{width:100%;height:170px;background:#0a0d11;border:1px solid #232b34;border-radius:6px;display:block}
.g{display:grid;grid-template-columns:repeat(4,1fr);gap:7px;margin-top:8px}
.t{background:#0a0d11;border:1px solid #222a33;border-radius:6px;padding:6px 8px}
.t b{display:block;font-size:10px;color:#7d8b9a;text-transform:uppercase}
.t span{font-family:ui-monospace,monospace;font-size:15px}
.sl{margin-bottom:14px}
.sl .hd{display:flex;justify-content:space-between;align-items:baseline}
.sl .nm{font-family:ui-monospace,monospace;font-size:13px}
.sl .vv{font-family:ui-monospace,monospace;font-size:16px;font-weight:700;color:#4fc3f7}
.sl .q{font-size:11px;color:#7d8b9a;margin-bottom:4px}
input[type=range]{width:100%;height:24px;-webkit-appearance:none;background:transparent}
input[type=range]::-webkit-slider-runnable-track{height:5px;background:#2b3742;border-radius:3px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;margin-top:-8px;
  border-radius:50%;background:#4fc3f7}
input[type=range]::-moz-range-track{height:5px;background:#2b3742;border-radius:3px}
input[type=range]::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:#4fc3f7;border:0}
button{padding:10px 14px;margin:0 6px 6px 0;background:#243040;color:#dfe7f0;border:1px solid #33455a;
       border-radius:6px;cursor:pointer;font-size:13px}
button:hover{background:#2c3b4e}
.on{background:#1f6b34;border-color:#2e9c4c}
.off{background:#7a2020;border-color:#a83030}
.bar{height:16px;background:#0a0d11;border-radius:3px;position:relative;overflow:hidden;margin:3px 0 7px}
.bar i{position:absolute;top:0;bottom:0;left:50%;background:#4fc3f7}
.lb{display:flex;justify-content:space-between;font-size:11px;color:#7d8b9a}
label.sw{display:inline-block;margin:0 12px 6px 0;font-size:13px;cursor:pointer}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:4px 0;border-bottom:1px solid #1c242c}
td:last-child{text-align:right;font-family:ui-monospace,monospace}
small{color:#7d8b9a}
</style></head><body>

<h1>PID Control Lab</h1>
<small>Two-wheel balancing robot &mdash; inverted pendulum</small>

<div class="card" style="margin-top:10px">
  <span class="st" id="st">--</span>
  <span class="mono" style="margin-left:10px">angle <b id="a">--</b>&deg;</span>
  <span class="mono" style="margin-left:10px">gyro <b id="gy">--</b>&deg;/s</span>
  <div style="margin-top:10px">
    <button class="on"  onclick="cmd('/start')">Start balancing</button>
    <button class="off" onclick="cmd('/stop')">Stop</button>
    <button onclick="cmd('/level')">Set upright zero</button>
  </div>
  <small>Hold it upright, press <b>Set upright zero</b>, then <b>Start balancing</b>.</small>
</div>

<h2>Step 0 &mdash; sensor check (do this first)</h2>
<div class="card">
  <div class="g" style="grid-template-columns:repeat(3,1fr)">
    <div class="t"><b>gyro X</b><span id="gx">--</span></div>
    <div class="t"><b>gyro Y</b><span id="gy2">--</span></div>
    <div class="t"><b>gyro Z</b><span id="gz">--</span></div>
    <div class="t"><b>accel X</b><span id="axv">--</span></div>
    <div class="t"><b>accel Y</b><span id="ayv">--</span></div>
    <div class="t"><b>accel Z</b><span id="azv">--</span></div>
    <div class="t"><b>raw tilt</b><span id="raw">--</span></div>
    <div class="t"><b>loop Hz</b><span id="hz">--</span></div>
    <div class="t"><b>imu errors</b><span id="err">--</span></div>
  </div>
  <small>Tip the robot forward. <b>Exactly one</b> gyro axis should swing hard &mdash;
  select it below. <b>raw tilt</b> must change smoothly and follow the lean.
  <b>loop Hz</b> must read about 100.</small>
  <div style="margin-top:9px">
    pitch gyro axis:
    <button onclick="set('gaxis',0)">X</button>
    <button onclick="set('gaxis',1)">Y</button>
    <button onclick="set('gaxis',2)">Z</button>
    <button onclick="flip('gsign')">flip gyro sign</button>
    <span class="mono" id="gsel"></span>
  </div>
  <div style="margin-top:6px">
    accel pair:
    <button onclick="set('apair',0)">X/Z</button>
    <button onclick="set('apair',1)">Y/Z</button>
    <button onclick="set('apair',2)">X/Y</button>
    <button onclick="flip('asign')">flip accel sign</button>
    <span class="mono" id="asel"></span>
  </div>
</div>

<h2>Step 0b &mdash; motor deadband</h2>
<div class="card">
  <small>Press <b>Stop</b> first. Raise the test output until the wheels
  <i>just</i> start turning &mdash; that number is your deadband. If it is much
  above 30, the robot cannot make small corrections and will feel sluggish and
  insensitive however you tune it.</small>
  <div class="sl" style="margin-top:8px"><div class="hd"><span class="nm">open-loop test</span>
    <span class="vv" id="vmt">0</span></div>
    <input type="range" id="smt" min="-160" max="160" step="5" value="0"></div>
  <div class="sl"><div class="hd"><span class="nm">deadband</span>
    <span class="vv" id="vdb">30</span></div>
    <input type="range" id="sdb" min="0" max="160" step="1" value="30"></div>
</div>

<h2>Response</h2>
<div class="card">
  <canvas id="cv" width="740" height="170"></canvas>
  <div class="lb"><span>&mdash; angle</span><span style="color:#81c784">&mdash; setpoint</span><span>12 s</span></div>
  <div style="margin-top:8px">
    <button onclick="cmd('/step')">Apply 3&deg; step</button>
  </div>
  <table style="margin-top:8px">
    <tr><td>Overshoot</td><td id="m_os">--</td></tr>
    <tr><td>Rise time</td><td id="m_rt">--</td></tr>
    <tr><td>Settling time</td><td id="m_st">--</td></tr>
    <tr><td>Steady-state error</td><td id="m_ss">--</td></tr>
  </table>
</div>

<h2>The three terms</h2>
<div class="card">
  <div class="lb"><span>P &mdash; how far off now</span><span id="tp">--</span></div>
  <div class="bar"><i id="bp"></i></div>
  <div class="lb"><span>I &mdash; how long off</span><span id="ti">--</span></div>
  <div class="bar"><i id="bi"></i></div>
  <div class="lb"><span>D &mdash; how fast changing</span><span id="td">--</span></div>
  <div class="bar"><i id="bd"></i></div>
  <div class="lb"><span><b>Motor command</b></span><span id="tm">--</span></div>
  <div class="bar"><i id="bm" style="background:#ffb74d"></i></div>
  <div style="margin-top:10px">
    <label class="sw"><input type="checkbox" id="useP" checked> use P</label>
    <label class="sw"><input type="checkbox" id="useI" checked> use I</label>
    <label class="sw"><input type="checkbox" id="useD" checked> use D</label>
  </div>
</div>

<h2>Gains</h2>
<div class="card">
  <div class="sl"><div class="hd"><span class="nm">Kp</span><span class="vv" id="vKp">0</span></div>
    <div class="q">PWM counts per degree of error</div>
    <input type="range" id="sKp" min="0" max="250" step="0.5" value="0"></div>
  <div class="sl"><div class="hd"><span class="nm">Ki</span><span class="vv" id="vKi">0</span></div>
    <div class="q">per degree-second of accumulated error</div>
    <input type="range" id="sKi" min="0" max="300" step="1" value="0"></div>
  <div class="sl"><div class="hd"><span class="nm">Kd</span><span class="vv" id="vKd">0</span></div>
    <div class="q">per degree-per-second of rate of change</div>
    <input type="range" id="sKd" min="0" max="20" step="0.05" value="0"></div>
</div>

<h2>Ziegler&ndash;Nichols</h2>
<div class="card">
  <small>Set Ki and Kd to zero. Raise Kp until the robot oscillates steadily &mdash;
  neither dying away nor growing. That Kp is the ultimate gain <b>Ku</b>, and the
  oscillation period is <b>Tu</b>.</small>
  <div class="g" style="grid-template-columns:repeat(2,1fr)">
    <div class="t"><b>measured period</b><span id="osc">--</span></div>
    <div class="t"><b>swing (pk-pk)</b><span id="amp">--</span></div>
  </div>
  <div style="margin-top:9px"><button onclick="cmd('/mark')">Record this as Ku, Tu</button></div>
  <table style="margin-top:8px">
    <tr><td>Ku</td><td id="ku">--</td></tr>
    <tr><td>Tu</td><td id="tu">--</td></tr>
  </table>
  <small>Classic tuning rules:</small>
  <table>
    <tr><td>P only &nbsp; Kp = 0.5&middot;Ku</td><td id="zn_p">--</td></tr>
    <tr><td>PI &nbsp; 0.45&middot;Ku, Tu/1.2</td><td id="zn_pi">--</td></tr>
    <tr><td>PID &nbsp; 0.6&middot;Ku, Tu/2, Tu/8</td><td id="zn_pid">--</td></tr>
  </table>
  <div><button onclick="applyZN()">Apply the PID row</button></div>
</div>

<h2>Implementation options</h2>
<div class="card">
  <label class="sw"><input type="checkbox" id="dom" checked> D from gyro, not d(error)/dt</label><br>
  <small>Unticking gives the textbook form: watch the noise and the kick when you step the setpoint.</small>
  <div style="margin-top:9px">
  <label class="sw"><input type="checkbox" id="aw" checked> anti-windup clamp</label><br>
  <small>Unticking lets the integral wind up freely. Hold the robot still and tilted to see it.</small></div>
  <div style="margin-top:10px"><button onclick="cmd('/reference')">Load reference gains</button></div>
</div>

<script>
const $=i=>document.getElementById(i);
function cmd(u){fetch(u)}
function set(k,v){fetch('/set?'+k+'='+v)}

let cur={};
function flip(k){ set(k, (cur[k]>0?-1:1)); }
['mt','db'].forEach(k=>{
  const s=$('s'+k), v=$('v'+k);
  s.addEventListener('input',()=>{v.textContent=s.value;set(k,s.value)});
});
['Kp','Ki','Kd'].forEach(k=>{
  const s=$('s'+k), v=$('v'+k);
  s.addEventListener('input',()=>{v.textContent=(+s.value).toFixed(2);set(k,s.value)});
});
['useP','useI','useD'].forEach(k=>$(k).addEventListener('change',e=>set(k,e.target.checked?1:0)));
$('dom').addEventListener('change',e=>set('dom',e.target.checked?1:0));
$('aw').addEventListener('change', e=>set('aw', e.target.checked?1:0));

let zn=null;
function applyZN(){ if(zn) fetch('/applyzn').then(()=>setTimeout(load,200)); }

function bar(el,val,scale){
  const f=Math.max(-1,Math.min(1,val/scale));
  el.style.left = (f<0 ? 50+f*50 : 50)+'%';
  el.style.width= Math.abs(f)*50+'%';
}

const cv=$('cv'), cx=cv.getContext('2d');
function draw(a,s){
  const W=cv.width,H=cv.height,N=a.length;
  cx.clearRect(0,0,W,H);
  cx.strokeStyle='#1a222b';
  for(let i=1;i<4;i++){const y=H*i/4;cx.beginPath();cx.moveTo(0,y);cx.lineTo(W,y);cx.stroke()}
  const SC=12;
  const plot=(arr,col)=>{cx.strokeStyle=col;cx.lineWidth=1.6;cx.beginPath();
    for(let i=0;i<N;i++){const y=H/2-Math.max(-1,Math.min(1,arr[i]/SC))*(H/2-5);
      i?cx.lineTo(i*W/N,y):cx.moveTo(0,y)}cx.stroke()};
  plot(s,'#81c784'); plot(a,'#4fc3f7');
}

const SC={BALANCING:'#1f6b34',STOPPED:'#7a5a10',FALLEN:'#7a2020'};
function load(){
  fetch('/data').then(r=>r.json()).then(d=>{
    $('st').textContent=d.state; $('st').style.background=SC[d.state]||'#33455a';
    $('a').textContent=d.angle.toFixed(2); $('gy').textContent=d.gyro.toFixed(1);
    $('tp').textContent=d.p.toFixed(1); $('ti').textContent=d.i.toFixed(1);
    $('td').textContent=d.d.toFixed(1); $('tm').textContent=d.cmd;
    bar($('bp'),d.p,255); bar($('bi'),d.i,255); bar($('bd'),d.d,255); bar($('bm'),d.cmd,255);
    $('osc').textContent=d.osc>0?d.osc.toFixed(2)+' s':'--';
    $('amp').textContent=d.amp.toFixed(1)+'°';
    $('ku').textContent=d.ku>0?d.ku.toFixed(1):'--';
    $('tu').textContent=d.tu>0?d.tu.toFixed(2)+' s':'--';
    if(d.ku>0&&d.tu>0){
      const kp=0.6*d.ku, ki=kp/(d.tu/2), kd=kp*(d.tu/8);
      zn={kp:kp,ki:ki,kd:kd};
      $('zn_p').textContent='Kp='+(0.5*d.ku).toFixed(1);
      $('zn_pi').textContent='Kp='+(0.45*d.ku).toFixed(1)+'  Ki='+((0.45*d.ku)/(d.tu/1.2)).toFixed(1);
      $('zn_pid').textContent='Kp='+kp.toFixed(1)+'  Ki='+ki.toFixed(1)+'  Kd='+kd.toFixed(2);
    }
    if(d.stepdone){
      const span=3.0;
      $('m_os').textContent=(Math.max(0,(d.peak-d.to)/span*100)).toFixed(0)+' %';
      $('m_rt').textContent=d.rise>0?d.rise.toFixed(2)+' s':'never reached';
      $('m_st').textContent=d.settle.toFixed(2)+' s';
      $('m_ss').textContent=d.ss.toFixed(2)+'°';
    }
    cur={gsign:d.gsign,asign:d.asign};
    $('gx').textContent=d.gx.toFixed(1); $('gy2').textContent=d.gy.toFixed(1);
    $('gz').textContent=d.gz.toFixed(1);
    $('axv').textContent=d.ax.toFixed(2); $('ayv').textContent=d.ay.toFixed(2);
    $('azv').textContent=d.az.toFixed(2);
    $('raw').textContent=d.raw.toFixed(1)+'°';
    $('hz').textContent=d.hz.toFixed(0); $('err').textContent=d.err;
    $('hz').style.color = (d.hz>90&&d.hz<110)?'#81c784':'#e57373';
    $('gsel').textContent='axis '+'XYZ'[d.gaxis]+(d.gsign<0?' (inverted)':'');
    $('asel').textContent=['X/Z','Y/Z','X/Y'][d.apair]+(d.asign<0?' (inverted)':'');
    if(document.activeElement.id!='smt'){$('smt').value=d.mt;$('vmt').textContent=d.mt;}
    if(document.activeElement.id!='sdb'){$('sdb').value=d.db;$('vdb').textContent=d.db;}
    $('sKp').value=d.Kp; $('vKp').textContent=d.Kp.toFixed(2);
    $('sKi').value=d.Ki; $('vKi').textContent=d.Ki.toFixed(2);
    $('sKd').value=d.Kd; $('vKd').textContent=d.Kd.toFixed(2);
    draw(d.tr,d.ts);
  }).catch(()=>{}).finally(()=>setTimeout(load,200));
}
load();
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleData() {
  String j = "{";
  j += "\"state\":\"" + String(balancing ? "BALANCING" : (fabs(angle) > 40 ? "FALLEN" : "STOPPED")) + "\",";
  j += "\"angle\":" + String(angle, 2) + ",\"gyro\":" + String(gyroRate, 1) + ",";
  j += "\"p\":" + String(useP ? pTerm : 0, 1) + ",\"i\":" + String(useI ? iTerm : 0, 1) + ",";
  j += "\"d\":" + String(useD ? dTerm : 0, 1) + ",\"cmd\":" + String(motorCmd) + ",";
  j += "\"Kp\":" + String(Kp, 2) + ",\"Ki\":" + String(Ki, 2) + ",\"Kd\":" + String(Kd, 3) + ",";
  j += "\"osc\":" + String(oscPeriod, 3) + ",\"amp\":" + String(oscAmplitude, 2) + ",";
  j += "\"ku\":" + String(Ku_recorded, 2) + ",\"tu\":" + String(Tu_recorded, 3) + ",";
  j += "\"stepdone\":" + String(stepDone ? 1 : 0) + ",\"peak\":" + String(stepPeak, 2) + ",";
  j += "\"to\":" + String(stepTo, 2) + ",\"rise\":" + String(stepRise, 2) + ",";
  j += "\"settle\":" + String(stepSettle, 2) + ",\"ss\":" + String(stepSS, 2) + ",";
  j += "\"hz\":" + String(loopHz, 1) + ",\"err\":" + String(imuErrors) + ",";
  j += "\"gx\":" + String((gyroRaw[0]-gyroOffset)/GYRO_LSB, 1) + ",";
  j += "\"gy\":" + String((gyroRaw[1]-gyroOffset)/GYRO_LSB, 1) + ",";
  j += "\"gz\":" + String((gyroRaw[2]-gyroOffset)/GYRO_LSB, 1) + ",";
  j += "\"ax\":" + String(accelX/8192.0, 2) + ",\"ay\":" + String(accelY/8192.0, 2) + ",";
  j += "\"az\":" + String(accelZ/8192.0, 2) + ",\"raw\":" + String(accelPitch(), 1) + ",";
  j += "\"gaxis\":" + String(PITCH_GYRO_AXIS) + ",\"gsign\":" + String((int)GYRO_SIGN) + ",";
  j += "\"apair\":" + String(ACCEL_PAIR) + ",\"asign\":" + String((int)ACCEL_SIGN) + ",";
  j += "\"db\":" + String(DEADBAND) + ",\"mt\":" + String(motorTest) + ",";

  j += "\"tr\":[";
  for (int n = 0; n < TRACE_N; n++) {
    j += String(traceAngle[(traceHead + n) % TRACE_N], 1);
    if (n < TRACE_N - 1) j += ",";
  }
  j += "],\"ts\":[";
  for (int n = 0; n < TRACE_N; n++) {
    j += String(traceSet[(traceHead + n) % TRACE_N], 1);
    if (n < TRACE_N - 1) j += ",";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleSet() {
  if (server.hasArg("Kp")) Kp = server.arg("Kp").toFloat();
  if (server.hasArg("Ki")) Ki = server.arg("Ki").toFloat();
  if (server.hasArg("Kd")) Kd = server.arg("Kd").toFloat();
  if (server.hasArg("useP")) useP = server.arg("useP") == "1";
  if (server.hasArg("useI")) useI = server.arg("useI") == "1";
  if (server.hasArg("useD")) useD = server.arg("useD") == "1";
  if (server.hasArg("dom")) dOnMeasurement = server.arg("dom") == "1";
  if (server.hasArg("aw"))  useAntiWindup  = server.arg("aw")  == "1";
  if (server.hasArg("gaxis")) PITCH_GYRO_AXIS = server.arg("gaxis").toInt();
  if (server.hasArg("gsign")) GYRO_SIGN  = server.arg("gsign").toFloat();
  if (server.hasArg("apair")) ACCEL_PAIR = server.arg("apair").toInt();
  if (server.hasArg("asign")) ACCEL_SIGN = server.arg("asign").toFloat();
  if (server.hasArg("db"))    DEADBAND   = server.arg("db").toInt();
  if (server.hasArg("mt"))    motorTest  = server.arg("mt").toInt();
  server.send(200, "text/plain", "ok");
}

void handleStart()  { motorTest = 0; integral = 0; prevError = 0; balancing = true;  server.send(200, "text/plain", "ok"); }
void handleStop()   { balancing = false; motorTest = 0; server.send(200, "text/plain", "ok"); }
void handleStep()   { startStepTest(); server.send(200, "text/plain", "ok"); }
void handleMark()   { Ku_recorded = Kp; Tu_recorded = oscPeriod; server.send(200, "text/plain", "ok"); }

void handleLevel() {
  // Take the accelerometer angle as the definition of upright. Do this while
  // holding the robot vertical, before starting.
  if (readIMU()) {
    angleTrim = accelPitch();
    prefs.putFloat("trim", angleTrim);
    angle = 0; kBias = 0;
  }
  server.send(200, "text/plain", "ok");
}

void handleApplyZN() {
  if (Ku_recorded > 0 && Tu_recorded > 0) {
    Kp = 0.6 * Ku_recorded;
    Ki = Kp / (Tu_recorded / 2.0);
    Kd = Kp * (Tu_recorded / 8.0);
  }
  server.send(200, "text/plain", "ok");
}

// A tune MEASURED on this robot, not a guess: found by sweeping gains over
// serial and scoring how long it stayed upright. It held 36.4 s unbroken with
// an angle standard deviation of 3.1 deg and no motor saturation at all.
//
// Note how low these are compared with a first guess. The instinct when a
// balance robot will not stand up is to raise Kp; on this machine every
// increase above ~15 made it worse, because the motor command was already
// saturating and the extra gain only fed the oscillation.
void handleReference() {
  Kp = 10.0; Ki = 0.0; Kd = 0.8;
  useP = useI = useD = true;
  setpoint = 0.0;
  server.send(200, "text/plain", "ok");
}

// ===========================================================================
// SECTION 10 - Setup
// ===========================================================================
void setup() {
  Serial.begin(115200);
  // Never let a full USB buffer stall the sketch when nothing is listening.
  Serial.setTxTimeoutMs(0);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  prefs.begin("pidlab", false);
  angleTrim = prefs.getFloat("trim", 0.0);

  // MPU-6050: wake, then set a sensible bandwidth and range. Leaving the
  // power-on defaults gives a 260 Hz bandwidth, which puts all the motor and
  // gearbox vibration straight into the gyro - and then Kd multiplies it.
  auto reg = [](uint8_t r, uint8_t v) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
    Wire.endTransmission(true);
  };
  reg(0x6B, 0x01);            // wake, PLL clock
  delay(10);
  reg(0x1A, 0x03);            // DLPF 44 Hz
  reg(0x19, 0x04);            // 200 Hz sample rate
  reg(0x1B, 0x08);            // gyro  +/-500 deg/s
  reg(0x1C, 0x08);            // accel +/-4 g  (was left at the +/-2 g default,
                              // so the reported |accel| read 2.18 g instead of 1)

  ledcAttach(PWM_LA, 20000, 8);
  ledcAttach(PWM_LB, 20000, 8);
  ledcAttach(PWM_RA, 20000, 8);
  ledcAttach(PWM_RB, 20000, 8);
  stopMotors();

  // Gyro zero offset: average it while the robot is held still.
  Serial.print("Calibrating gyro, hold still... ");
  float sum = 0; int n = 0;
  for (int i = 0; i < 400; i++) {
    if (readIMU()) { sum += gyroRaw[PITCH_GYRO_AXIS]; n++; }
    delay(3);
  }
  if (n) gyroOffset = sum / n;
  Serial.println("done");

  if (readIMU()) angle = accelPitch() - angleTrim;

  for (int i = 0; i < TRACE_N; i++) { traceAngle[i] = 0; traceSet[i] = 0; }

  WiFi.softAP("PID-Lab", "");
  Serial.print("Open http://"); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set", handleSet);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/step", handleStep);
  server.on("/mark", handleMark);
  server.on("/level", handleLevel);
  server.on("/applyzn", handleApplyZN);
  server.on("/reference", handleReference);
  server.begin();

  xTaskCreatePinnedToCore(controlLoop, "control", 8192, NULL, 5, NULL, 1);
}

// --- serial command interface -------------------------------------------
// Same actions as the web page, driveable from a terminal over USB. Useful for
// logging a whole tuning run, and for scripting a search over gains.
//   P 45 | I 90 | D 2.2 | GO | HALT | ZERO | DB 40 | TEST 60
//   GAXIS 0|1|2 | GSIGN -1 | APAIR 0|1|2 | ASIGN -1 | SP 0.0
void handleSerial() {
  static char buf[48];
  static uint8_t n = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n == 0) continue;
      buf[n] = 0; n = 0;

      char cmd[16]; float v = 0;
      int got = sscanf(buf, "%15s %f", cmd, &v);
      if (got < 1) continue;
      for (char *q = cmd; *q; q++) *q = toupper(*q);

      if      (!strcmp(cmd, "P"))     Kp = v;
      else if (!strcmp(cmd, "I"))     Ki = v;
      else if (!strcmp(cmd, "D"))     Kd = v;
      else if (!strcmp(cmd, "SP"))    setpoint = v;
      else if (!strcmp(cmd, "DB"))    DEADBAND = (int)v;
      else if (!strcmp(cmd, "RAMP"))  RAMPWIN  = (int)v;
      else if (!strcmp(cmd, "PWM"))   { PWMFREQ = (int)v;
          ledcChangeFrequency(PWM_LA, PWMFREQ, 8); ledcChangeFrequency(PWM_LB, PWMFREQ, 8);
          ledcChangeFrequency(PWM_RA, PWMFREQ, 8); ledcChangeFrequency(PWM_RB, PWMFREQ, 8); }
      else if (!strcmp(cmd, "TEST"))  { balancing = false; motorTest = (int)v; }
      else if (!strcmp(cmd, "GAXIS")) PITCH_GYRO_AXIS = (int)v;
      else if (!strcmp(cmd, "GSIGN")) GYRO_SIGN = v;
      else if (!strcmp(cmd, "APAIR")) ACCEL_PAIR = (int)v;
      else if (!strcmp(cmd, "ASIGN")) ACCEL_SIGN = v;
      else if (!strcmp(cmd, "GO"))    { motorTest = 0; integral = 0; prevError = 0; balancing = true; }
      else if (!strcmp(cmd, "HALT"))  { balancing = false; motorTest = 0; }
      else if (!strcmp(cmd, "TRIM"))  { angleTrim += v; prefs.putFloat("trim", angleTrim);
                                        integral = 0; }
      else if (!strcmp(cmd, "ZERO"))  { angleTrim = accelPitch(); prefs.putFloat("trim", angleTrim);
                                        angle = 0; kBias = 0; integral = 0; }
      else { Serial.printf("ACK,?,%s\n", cmd); continue; }

      Serial.printf("ACK,%s,%.3f\n", cmd, v);
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void loop() {
  handleSerial();
  server.handleClient();

  // Session telemetry at 5 Hz, so a run can be logged and reviewed afterwards
  // (and so someone on the USB cable can watch the tuning as it happens).
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 20) {      // 50 Hz: fast enough to resolve the oscillation
    lastLog = millis();
    Serial.printf("T,%lu,%s,%.2f,%.1f,%.2f,%.1f,%.1f,%.1f,%d,%.1f,%.1f,%.2f,%d,%d,%d,%d,%.0f,%lu\n",
      millis(),
      balancing ? "BAL" : (fabs(angle) > 40 ? "FALL" : "STOP"),
      angle, gyroRate, setpoint,
      useP ? pTerm : 0.0f, useI ? iTerm : 0.0f, useD ? dTerm : 0.0f,
      motorCmd, Kp, Ki, Kd,
      useP ? 1 : 0, useI ? 1 : 0, useD ? 1 : 0,
      DEADBAND, loopHz, imuErrors);
  }

  delay(2);
}
