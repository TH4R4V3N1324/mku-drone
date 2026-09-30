#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "imu.h"
#include "motorMixer.h"
#include "motor.h"
#include "pid.h"
#include "receiver.h"
#include "serialTuner.h"

#define TUNE_PID 0        // Set to 1 to enable PID tuning via serial commands
#define CALIBRATE_ESCS 0  // Set to 1 to calibrate ESCs on startup (ensure props are removed) 

constexpr float MAX_THROTTLE = 0.85f; // 1850 us, leaving correction headroom

void run();
void test();
#if CALIBRATE_ESCS
void calibrateEscs();
#endif

Motor motor1(4); // PWM pin for motor 1
Motor motor2(5); // PWM pin for motor 2
Motor motor3(6); // PWM pin for motor 3
Motor motor4(7); // PWM pin for motor 4
MotorMixer mixer(motor1, motor2, motor3, motor4);
IMU imu;

constexpr uint8_t throttlePin = 9;
constexpr uint8_t rollPin     = 11;
constexpr uint8_t pitchPin    = 10;
constexpr uint8_t yawPin      = 8;
constexpr uint8_t aux1Pin     = 12;
constexpr uint8_t aux2Pin     = 13;
Receiver receiver(throttlePin, rollPin, pitchPin, yawPin, aux1Pin, aux2Pin);

constexpr float HOVER_CORRECTION_LIMIT = 0.25f;  // PID output range (+/-)
constexpr float HOVER_BAND             = 0.25f;  // final throttle stays within hoverThrottle +/- this
constexpr float HOVER_THROTTLE_MIN     = 0.15f;  // floor in hover (must be above the 0.05 motor cutoff)
constexpr float HOVER_SLEW_RATE        = 0.3f;   // max throttle change per second
constexpr float HOVER_MIN_TILT_COS     = 0.7f;   // limits tilt compensation to ~45 degrees
constexpr float ENTRY_MAX_VZ           = 0.3f;   // m/s, must be below this to engage
constexpr float ENTRY_HOLD_S           = 0.5f;   // seconds the conditions must hold
constexpr float ENTRY_MIN_THROTTLE     = 0.2f;   // can't engage hover on the ground
constexpr float CLIMB_DEADBAND         = 0.1f;   // m/s
constexpr float THROTTLE_FILTER_TAU    = 0.3f;   // s, smoothing of stick throttle for capture
constexpr float ACCEL_FILTER_HZ        = 20.0f;  // low-pass on vertical accel
constexpr float VZ_LEAK_TAU            = 2.0f;   // s, drift decay time constant
constexpr float GRAVITY                = 9.80665f; // m/s^2

static bool  hoverActive    = false;
static float vz             = 0.0f;
static float azFilt         = 0.0f;
static float throttleFilt   = 0.0f;
static float hoverThrottle  = 0.0f;
static float hoverCmd       = 0.0f;
static float steadyTime     = 0.0f;

PID altitudePID(0.1, 0.0, 0.0, -HOVER_CORRECTION_LIMIT, HOVER_CORRECTION_LIMIT); // PID controller for altitude
PID rollPID(0.02, 0.0, 0.0, -0.3, 0.3); // PID controller for roll
PID pitchPID(0.02, 0.0, 0.0, -0.3, 0.3); // PID controller for pitch
PID yawPID(0.05, 0.0, 0.0, -0.2, 0.2); // PID controller for yaw
SerialTuner tuner(rollPID, pitchPID, yawPID, altitudePID);

unsigned long previousLoopTime;

void setup() {
  Serial.begin(115200);
  #if CALIBRATE_ESCS
    calibrateEscs();
  #else
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz
    mixer.beginAllMotors();
    imu.init();
    imu.calibrate();
    receiver.init();
    mixer.stopAllMotors();
    delay(2000);
    previousLoopTime = micros();
  #endif
}

void loop() {
  if (TUNE_PID) {tuner.update();}
  run();
  //test();
}

/**
 * @brief Main control loop for the drone
 * @details Reads data from the IMU and receiver, computes PID outputs, and mixes motor speeds
 * @return None
 */
void run() {
  unsigned long now = micros();
  float dt = (now - previousLoopTime) / 1000000.0f;
  previousLoopTime = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  imu.readData();
  receiver.readData();

  float stickThrottle = receiver.getThrottle();
  bool hoverSwitch = receiver.getAux1() > 0.5f;

  // Filtered stick throttle (used for hover capture)
  throttleFilt += (dt / (THROTTLE_FILTER_TAU + dt)) * (stickThrottle - throttleFilt);

  // Vertical velocity estimate (runs every loop so it's valid on entry)
  float rollRad  = imu.getRoll()  * DEG_TO_RAD;
  float pitchRad = imu.getPitch() * DEG_TO_RAD;
  float tiltCos  = cosf(rollRad) * cosf(pitchRad);

  float azWorld = imu.getAccelZ() * tiltCos - GRAVITY;   // flip sign if your Z axis points down
  const float accelTau = 1.0f / (2.0f * PI * ACCEL_FILTER_HZ);
  azFilt += (dt / (accelTau + dt)) * (azWorld - azFilt);

  vz += azFilt * dt;
  vz -= vz * dt / VZ_LEAK_TAU;                           // dt-independent leak
  if (!hoverActive && stickThrottle <= 0.05f) vz = 0.0f; // on the ground / idle

  // Hover entry / exit
  if (!hoverSwitch) {
    hoverActive = false;
    steadyTime = 0.0f;
  } else if (!hoverActive) {
    bool steady = fabsf(vz) < ENTRY_MAX_VZ && throttleFilt > ENTRY_MIN_THROTTLE;
    steadyTime = steady ? steadyTime + dt : 0.0f;

    if (steadyTime >= ENTRY_HOLD_S) {
      hoverActive   = true;
      hoverThrottle = throttleFilt;   // captured hover throttle
      hoverCmd      = hoverThrottle;  // start the slew limiter here (bumpless)
      vz            = 0.0f;
      altitudePID.reset();
    }
  }

  // Throttle
  float throttle;
  float hoverSetpoint = 0.0f;
  float hoverMeasured = vz;
  float hoverCorrection = 0.0f;
  if (hoverActive) {
    float climbRate = receiver.getAux2() * 2.0f - 1.0f;  // +/-1 m/s
    if (fabsf(climbRate) < CLIMB_DEADBAND) climbRate = 0.0f;

    hoverSetpoint = climbRate;
    hoverCorrection = altitudePID.compute(hoverSetpoint, hoverMeasured, dt);

    // Clamp to a band around the captured throttle, above the floor
    float lower  = fmaxf(hoverThrottle - HOVER_BAND, HOVER_THROTTLE_MIN);
    float upper  = hoverThrottle + HOVER_BAND;
    float target = constrain(hoverThrottle + hoverCorrection, lower, upper);

    // Slew limit
    float maxStep = HOVER_SLEW_RATE * dt;
    hoverCmd += constrain(target - hoverCmd, -maxStep, maxStep);

    // Tilt compensation
    throttle = hoverCmd / fmaxf(tiltCos, HOVER_MIN_TILT_COS);
    throttle = constrain(throttle, HOVER_THROTTLE_MIN, MAX_THROTTLE);
  } else {
    throttle = stickThrottle;
    if (throttle <= 0.05f) {
      mixer.stopAllMotors();
      return;
    }
    throttle = constrain(throttle, 0.0f, MAX_THROTTLE);
  }

  float rollSetpoint = receiver.getRoll() * 30.0f;
  float pitchSetpoint = receiver.getPitch() * 30.0f;
  float yawRateSetpoint = receiver.getYaw() * 180.0f;

  float rollOutput = rollPID.compute(rollSetpoint, imu.getRoll(), dt);
  float pitchOutput = pitchPID.compute(pitchSetpoint, imu.getPitch(), dt);
  float yawOutput = yawPID.compute(yawRateSetpoint, imu.getGyroZ(), dt);

  if (TUNE_PID) {
    const char* axis = tuner.getActiveAxisName();
    float setpointToPrint, measuredToPrint, outputToPrint;
    if (strcmp(axis, "roll") == 0) {
      setpointToPrint = rollSetpoint;
      measuredToPrint = imu.getRoll();
      outputToPrint = rollOutput;
    } else if (strcmp(axis, "pitch") == 0) {
      setpointToPrint = pitchSetpoint;
      measuredToPrint = imu.getPitch();
      outputToPrint = pitchOutput;
    } else if (strcmp(axis, "yaw") == 0) {
      setpointToPrint = yawRateSetpoint;
      measuredToPrint = imu.getGyroZ();
      outputToPrint = yawOutput;
    } else if (strcmp(axis, "hover") == 0) {
      setpointToPrint = hoverSetpoint;
      measuredToPrint = hoverMeasured;
      outputToPrint = hoverCorrection;
    } else {
      setpointToPrint = 0.0f;
      measuredToPrint = 0.0f;
      outputToPrint = 0.0f;
    }

    //Plot with Teleplot to tune PID parameters
    Serial.print(">Setpoint: "); Serial.println(setpointToPrint);
    Serial.print(">Actual: "); Serial.println(measuredToPrint);
    Serial.print(">Output: "); Serial.println(outputToPrint);
  }

  mixer.mixMotors(throttle, rollOutput, pitchOutput, yawOutput);
}

/**
 * @brief Test the receiver-to-mixer path without running the PID controllers
 * @details Uses the transmitter throttle and small normalized attitude commands
 * so each stick can be checked independently with the propellers removed.
 * @return None
 */
void test() {
  receiver.readData();
  receiver.printData();

  float throttle = receiver.getThrottle();
  float roll = receiver.getRoll() * 0.20f;
  float pitch = receiver.getPitch() * 0.20f;
  float yaw = receiver.getYaw() * 0.20f;

  if (throttle <= 0.05f) {
    mixer.stopAllMotors();
    return;
  }

  mixer.mixMotors(throttle, roll, pitch, yaw);
}

#if CALIBRATE_ESCS
/**
 * @brief Calibrate the ESCs by sending a high throttle signal followed by a low throttle signal
 * @details This function should be called once on startup with the propellers removed.
 * It sends a high throttle signal for 2 seconds, then a low throttle signal for 2 seconds.
 * @return None
 */
void calibrateEscs() {
  mixer.beginAllMotors();

  Serial.println(F("ESC calibration: props OFF."));
  Serial.println(F("Sending MAX throttle — power the ESCs now."));
  unsigned long start = millis();
  while (millis() - start < 15000) {
    motor1.setSpeed(1.0f); motor2.setSpeed(1.0f);
    motor3.setSpeed(1.0f); motor4.setSpeed(1.0f);
  }

  Serial.println(F("Sending MIN throttle — listen for confirmation beeps."));
  start = millis();
  while (millis() - start < 10000) {
    motor1.setSpeed(0.0f); motor2.setSpeed(0.0f);
    motor3.setSpeed(0.0f); motor4.setSpeed(0.0f);
  }

  Serial.println(F("Calibration pulses sent. Halting — power-cycle to arm normally."));
  mixer.stopAllMotors();
  while (true) {}   // stop here on purpose, don't fall into run()
}
#endif