#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "imu.h"
#include "motorMixer.h"
#include "escOutput.h"
#include "pid.h"
#include "receiver.h"
#include "serialTuner.h"
#include "calibrationStore.h"

#define TUNE_PID 0        // Set to 1 to enable PID tuning via serial commands
#define CALIBRATE_ESCS 0  // Set to 1 to calibrate ESCs on startup (ensure props are removed)
#define CALIBRATE_IMU 0   // Set to 1 to run the guided IMU level calibration and save it to EEPROM

#if CALIBRATE_ESCS && CALIBRATE_IMU
#error "Enable only one of CALIBRATE_ESCS and CALIBRATE_IMU"
#endif

constexpr float MAX_THROTTLE = 0.85f; // 1850 us, leaving correction headroom

void run();
void disarm();
void test();
[[noreturn]] void haltWithImuError();
#if CALIBRATE_ESCS
void calibrateEscs();
#endif
#if CALIBRATE_IMU
void calibrateImuLevel();
#endif

// ESC signal pins: front left, front right, rear right, rear left (must be on PORTD)
constexpr uint8_t motorPins[ESC_COUNT] = {4, 5, 6, 7};
EscOutput esc;
MotorMixer mixer(esc);
IMU imu;

constexpr uint8_t throttlePin = 9;
constexpr uint8_t rollPin     = 11;
constexpr uint8_t pitchPin    = 10;
constexpr uint8_t yawPin      = 8;
constexpr uint8_t aux1Pin     = 12;
constexpr uint8_t aux2Pin     = 13;
Receiver receiver(throttlePin, rollPin, pitchPin, yawPin, aux1Pin, aux2Pin);

constexpr float ROLL_CORRECTION_LIMIT  = 0.3f;   // PID output range (+/-)
constexpr float PITCH_CORRECTION_LIMIT = 0.3f;   // PID output range (+/-)
constexpr float YAW_CORRECTION_LIMIT   = 0.2f;   // PID output range (+/-)
constexpr float MAX_TILT_ANGLE         = 30.0f;  // deg, full stick roll/pitch
constexpr float MAX_TILT_RATE          = 200.0f; // deg/s, outer angle loop output limit
constexpr float MAX_YAW_RATE           = 180.0f; // deg/s, full stick yaw
constexpr float RATE_D_FILTER_HZ       = 30.0f;  // low-pass on the rate loops' D term only

// Fine level adjustment on top of the EEPROM calibration. If it drifts in hover, trim in
// ~0.5 deg steps; positive values act like holding a little positive roll/pitch stick.
constexpr float ROLL_TRIM_DEG          = 0.0f;
constexpr float PITCH_TRIM_DEG         = 0.0f;
constexpr uint16_t LEVEL_BOOT_SAMPLES  = 2000;   // fallback level measurement at boot if EEPROM is empty (~3 s)

constexpr float HOVER_CORRECTION_LIMIT = 0.25f;  // PID output range (+/-)
constexpr float HOVER_BAND             = 0.25f;  // final throttle stays within hoverThrottle +/- this
constexpr float HOVER_THROTTLE_MIN     = 0.15f;  // floor in hover (must be above the 0.05 motor cutoff)
constexpr float HOVER_SLEW_RATE        = 0.3f;   // max throttle change per second
constexpr float THROTTLE_HANDOFF_RATE  = 1.5f;   // max manual throttle handoff change per second
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
static float handoffThrottle = 0.0f;
static bool  throttleHandoff = false;
static float steadyTime     = 0.0f;

constexpr float   ARM_THROTTLE      = 0.08f; // stick above this arms (once unblocked)
constexpr float   IDLE_THROTTLE     = 0.03f; // stick at/below this disarms and unblocks arming
constexpr uint8_t IMU_FAIL_LIMIT    = 25;    // consecutive failed IMU reads (100 ms at 250 Hz) before disarming
constexpr float   CRASH_ANGLE_DEG   = 45.0f; // roll or pitch beyond this means crashed or stuck (max commanded tilt is 30)
constexpr float   CRASH_TIME_S      = 0.1f;  // how long the tilt must last before cutting the motors

static bool    throttleIdle = true;  // motors stopped (disarmed)
static bool    armBlocked   = true;  // must see a valid low throttle before arming (boot, signal loss, IMU failure, crash)
static uint8_t imuFailCount = 0;
static float   crashTime    = 0.0f;  // s spent beyond CRASH_ANGLE_DEG while armed

PID altitudePID(0.1, 0.0, 0.0, -HOVER_CORRECTION_LIMIT, HOVER_CORRECTION_LIMIT); // PID controller for altitude
// Cascaded attitude control: angle error (deg) -> outer P -> rate setpoint (deg/s) -> inner PID on gyro -> motor correction.
// Roll/pitch gains tuned on a single-axis test rig; yaw is not tuned yet.
PID rollAnglePID(3.0 , 0.0, 0.0, -MAX_TILT_RATE, MAX_TILT_RATE); // outer loop: roll angle -> roll rate setpoint
PID pitchAnglePID(3.0, 0.0, 0.0, -MAX_TILT_RATE, MAX_TILT_RATE); // outer loop: pitch angle -> pitch rate setpoint
PID rollRatePID(0.0045, 0.0008, 0.00002, -ROLL_CORRECTION_LIMIT, ROLL_CORRECTION_LIMIT, RATE_D_FILTER_HZ); // inner loop: roll rate
PID pitchRatePID(0.0045, 0.0008, 0.00002, -PITCH_CORRECTION_LIMIT, PITCH_CORRECTION_LIMIT, RATE_D_FILTER_HZ); // inner loop: pitch rate
PID yawPID(0.004, 0.002, 0.0, -YAW_CORRECTION_LIMIT, YAW_CORRECTION_LIMIT, RATE_D_FILTER_HZ); // yaw rate
SerialTuner tuner(rollAnglePID, rollRatePID, pitchAnglePID, pitchRatePID, yawPID, altitudePID);

unsigned long previousLoopTime;
unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200);
  #if CALIBRATE_ESCS
    calibrateEscs();
  #else
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz
    Wire.setWireTimeout(3000, true); // Set I2C timeout to 3 ms (value is in us) and reset the bus on timeout

    imu.init();
    #if CALIBRATE_IMU
      calibrateImuLevel(); // does not return
    #endif

    // Gyro bias drifts with temperature, so measure it every boot (works on uneven ground)
    if (!imu.calibrateGyro()) {
      haltWithImuError();
    }

    // Level offsets depend on how the IMU is mounted, so they come from EEPROM
    IMU::LevelCalibration level;
    if (loadLevelCalibration(level)) {
      Serial.println(F("Loaded IMU level calibration from EEPROM."));
    } else {
      Serial.println(F("No saved IMU level calibration - measuring level now, keep the frame level."));
      Serial.println(F("Run CALIBRATE_IMU once for an accurate, repeatable calibration."));
      bool moved = true;
      while (moved) {
        if (!imu.measureLevel(level, LEVEL_BOOT_SAMPLES, moved)) {
          haltWithImuError();
        }
        if (moved) {
          Serial.println(F("Movement detected, retrying - keep the drone still."));
        }
      }
    }
    imu.setLevelCalibration(level);
    if (!imu.resetOrientation()) {
      haltWithImuError();
    }

    receiver.init();
    esc.begin(motorPins); // Timer1 keeps sending min-throttle pulses from here on
    delay(2000);
    mixer.stopAllMotors();
    previousLoopTime = micros();
  #endif
}

void loop() {
  esc.waitForNextCycle(); // fixed ESC_UPDATE_RATE_HZ control loop
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
  if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / ESC_UPDATE_RATE_HZ;

  receiver.readData();

  // Keep flying on the last sample through brief I2C glitches, but disarm if the IMU stays unreachable
  if (imu.readData()) {
    imuFailCount = 0;
  } else if (imuFailCount < IMU_FAIL_LIMIT) {
    ++imuFailCount;
  }
  if (imuFailCount >= IMU_FAIL_LIMIT) {
    disarm();
    return;
  }

  // Crash cutoff: a tilt far past anything the sticks can command means flipped, hit or stuck, in any mode
  const bool tooSteep = fabsf(imu.getRoll()) > CRASH_ANGLE_DEG || fabsf(imu.getPitch()) > CRASH_ANGLE_DEG;
  crashTime = (tooSteep && !throttleIdle) ? crashTime + dt : 0.0f;
  if (crashTime >= CRASH_TIME_S) {
    Serial.println(F("Tilt limit exceeded - motors cut. Lower the throttle to zero to re-arm."));
    crashTime = 0.0f;
    disarm();
    return;
  }

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
    if (hoverActive) {
      handoffThrottle = hoverCmd / fmaxf(tiltCos, HOVER_MIN_TILT_COS);
      handoffThrottle = constrain(handoffThrottle, HOVER_THROTTLE_MIN, MAX_THROTTLE);
      throttleHandoff = true;
    }
    hoverActive = false;
    steadyTime = 0.0f;
  } else if (!hoverActive) {
    bool steady = !throttleIdle && fabsf(vz) < ENTRY_MAX_VZ && throttleFilt > ENTRY_MIN_THROTTLE; // never engage while disarmed
    steadyTime = steady ? steadyTime + dt : 0.0f;

    if (steadyTime >= ENTRY_HOLD_S) {
      hoverActive   = true;
      hoverThrottle = throttleFilt;   // captured hover throttle
      hoverCmd      = hoverThrottle;  // start the slew limiter here (bumpless)
      throttleHandoff = false;
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
    if (throttleIdle) {
      // Only arm after a live link has shown the throttle low, so booting or reconnecting with the stick up can't spin the motors
      if (!receiver.hasSignal()) {
        armBlocked = true;
      } else if (stickThrottle <= IDLE_THROTTLE) {
        armBlocked = false;
      }

      if (!armBlocked && stickThrottle > ARM_THROTTLE) {
        rollAnglePID.reset();
        pitchAnglePID.reset();
        rollRatePID.reset();
        pitchRatePID.reset();
        yawPID.reset();
        altitudePID.reset();
        throttleIdle = false;
      }
    } else {
      if (stickThrottle <= IDLE_THROTTLE) throttleIdle = true;
    }

    if (throttleIdle) {
      throttleHandoff = false;
      mixer.stopAllMotors();
      return;
    }

    if (throttleHandoff) {
      float maxStep = THROTTLE_HANDOFF_RATE * dt;
      handoffThrottle += constrain(stickThrottle - handoffThrottle, -maxStep, maxStep);
      throttle = handoffThrottle;
      if (fabsf(stickThrottle - handoffThrottle) <= maxStep) {
        throttleHandoff = false;
      }
    } else {
      throttle = stickThrottle;
    }
    throttle = constrain(throttle, 0.0f, MAX_THROTTLE);
  }

  float rollSetpoint = receiver.getRoll() * MAX_TILT_ANGLE + ROLL_TRIM_DEG;
  float pitchSetpoint = receiver.getPitch() * MAX_TILT_ANGLE + PITCH_TRIM_DEG;
  float yawRateSetpoint = receiver.getYaw() * MAX_YAW_RATE;

  // Outer loop: angle error -> desired rotation rate
  float rollRateSetpoint = rollAnglePID.compute(rollSetpoint, imu.getRoll(), dt);
  float pitchRateSetpoint = pitchAnglePID.compute(pitchSetpoint, imu.getPitch(), dt);

  // Inner loop: track the rate with the gyro directly (roll integrates gyro Y, pitch gyro X)
  float rollOutput = rollRatePID.compute(rollRateSetpoint, imu.getGyroY(), dt);
  float pitchOutput = pitchRatePID.compute(pitchRateSetpoint, imu.getGyroX(), dt);
  float yawOutput = yawPID.compute(yawRateSetpoint, imu.getGyroZ(), dt);

  if (TUNE_PID && millis() - lastTelemetryTime >= 50) {
    lastTelemetryTime = millis();
    const char* axis = tuner.getActiveAxisName();
    float setpointToPrint, measuredToPrint, outputToPrint;
    if (strcmp(axis, "roll") == 0) {
      setpointToPrint = rollSetpoint;
      measuredToPrint = imu.getRoll();
      outputToPrint = rollRateSetpoint;
    } else if (strcmp(axis, "rollrate") == 0) {
      setpointToPrint = rollRateSetpoint;
      measuredToPrint = imu.getGyroY();
      outputToPrint = rollOutput;
    } else if (strcmp(axis, "pitch") == 0) {
      setpointToPrint = pitchSetpoint;
      measuredToPrint = imu.getPitch();
      outputToPrint = pitchRateSetpoint;
    } else if (strcmp(axis, "pitchrate") == 0) {
      setpointToPrint = pitchRateSetpoint;
      measuredToPrint = imu.getGyroX();
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
    Serial.print(">Setpoint:"); Serial.println(setpointToPrint);
    Serial.print(">Actual:"); Serial.println(measuredToPrint);
    Serial.print(">Output:"); Serial.println(outputToPrint);
  }

  mixer.mixMotors(throttle, rollOutput, pitchOutput, yawOutput);
}

/**
 * @brief Stop the motors and require a valid low throttle before arming again
 * @details Also leaves hover mode so it can't re-engage until re-armed.
 * @return None
 */
void disarm() {
  hoverActive     = false;
  throttleHandoff = false;
  steadyTime      = 0.0f;
  throttleIdle    = true;
  armBlocked      = true;
  mixer.stopAllMotors();
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
 * It sends a high throttle signal for 15 seconds (power the ESCs during this), then a low
 * throttle signal for 10 seconds, then halts.
 * @return None
 */
void calibrateEscs() {
  esc.begin(motorPins);

  Serial.println(F("ESC calibration: props OFF."));
  Serial.println(F("Sending MAX throttle — power the ESCs now."));
  unsigned long start = millis();
  while (millis() - start < 15000) {
    mixer.setAllMotors(1.0f); // keep refreshing so the output watchdog doesn't drop to min
  }

  Serial.println(F("Sending MIN throttle — listen for confirmation beeps."));
  start = millis();
  while (millis() - start < 10000) {
    mixer.setAllMotors(0.0f);
  }

  Serial.println(F("Calibration pulses sent. Halting — power-cycle to arm normally."));
  mixer.stopAllMotors();
  while (true) {}   // stop here on purpose, don't fall into run()
}
#endif

/**
 * @brief Report an IMU I2C failure and halt
 * @details Called before the ESC output starts, so the ESCs stay unarmed with no signal.
 * @return Never returns
 */
void haltWithImuError() {
  Serial.println(F("IMU I2C error. Check wiring and power-cycle."));
  while (true) {}
}

#if CALIBRATE_IMU
/**
 * @brief Block until a key is sent over Serial, then discard the rest of the line
 * @return None
 */
static void waitForKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}
  delay(50);
  while (Serial.available()) Serial.read();
}

/**
 * @brief Guided IMU level calibration, saved to EEPROM
 * @details Measures level several times, with the drone picked up and set down between
 * rounds, and only saves the average if the rounds agree. Rounds where the drone moved are
 * repeated. Open the serial monitor at 115200 baud; ESCs receive no signal while this runs.
 * @return Never returns
 */
void calibrateImuLevel() {
  constexpr uint8_t ROUNDS = 5;
  constexpr uint16_t SAMPLES_PER_ROUND = 4000; // ~6 s per round
  constexpr float MAX_SPREAD_DEG = 0.3f;       // rounds must agree within this to be saved

  Serial.println(F("IMU level calibration."));
  Serial.println(F("Put a spirit level on the FRAME (not the table) and shim it level."));
  Serial.println(F("Measuring gyro bias - keep it still..."));
  if (!imu.calibrateGyro()) {
    haltWithImuError();
  }

  IMU::LevelCalibration rounds[ROUNDS];
  uint8_t round = 0;
  while (round < ROUNDS) {
    Serial.print(F("Round ")); Serial.print(round + 1); Serial.print('/'); Serial.print(ROUNDS);
    Serial.println(F(": set the drone down level and still, then send any key."));
    waitForKey();
    Serial.println(F("Measuring - don't touch it..."));

    bool moved;
    if (!imu.measureLevel(rounds[round], SAMPLES_PER_ROUND, moved)) {
      haltWithImuError();
    }
    if (moved) {
      Serial.println(F("Movement detected - repeating this round."));
      continue;
    }

    Serial.print(F("  pitch ")); Serial.print(rounds[round].pitchOffset, 3);
    Serial.print(F(" deg | roll ")); Serial.print(rounds[round].rollOffset, 3);
    Serial.print(F(" deg | accelZ offset ")); Serial.print(rounds[round].accelZOffset, 3);
    Serial.println(F(" m/s^2"));
    ++round;
    if (round < ROUNDS) {
      Serial.println(F("Pick it up, tilt it around, then set it back down level."));
    }
  }

  IMU::LevelCalibration mean = {0.0f, 0.0f, 0.0f};
  float minPitch = rounds[0].pitchOffset, maxPitch = minPitch;
  float minRoll = rounds[0].rollOffset, maxRoll = minRoll;
  for (uint8_t i = 0; i < ROUNDS; ++i) {
    mean.pitchOffset += rounds[i].pitchOffset / ROUNDS;
    mean.rollOffset += rounds[i].rollOffset / ROUNDS;
    mean.accelZOffset += rounds[i].accelZOffset / ROUNDS;
    minPitch = fminf(minPitch, rounds[i].pitchOffset);
    maxPitch = fmaxf(maxPitch, rounds[i].pitchOffset);
    minRoll = fminf(minRoll, rounds[i].rollOffset);
    maxRoll = fmaxf(maxRoll, rounds[i].rollOffset);
  }
  const float pitchSpread = maxPitch - minPitch;
  const float rollSpread = maxRoll - minRoll;

  Serial.print(F("Average: pitch ")); Serial.print(mean.pitchOffset, 3);
  Serial.print(F(" deg | roll ")); Serial.print(mean.rollOffset, 3);
  Serial.print(F(" deg | accelZ offset ")); Serial.print(mean.accelZOffset, 3);
  Serial.println(F(" m/s^2"));
  Serial.print(F("Spread between rounds: pitch ")); Serial.print(pitchSpread, 3);
  Serial.print(F(" deg | roll ")); Serial.print(rollSpread, 3);
  Serial.println(F(" deg"));

  if (pitchSpread > MAX_SPREAD_DEG || rollSpread > MAX_SPREAD_DEG) {
    Serial.println(F("Rounds disagree - NOT saved. Check the surface is solid and the IMU mount isn't loose, then retry."));
  } else {
    saveLevelCalibration(mean);
    IMU::LevelCalibration check;
    if (loadLevelCalibration(check)) {
      Serial.println(F("Saved to EEPROM and verified."));
    } else {
      Serial.println(F("Save FAILED verification - offsets out of range? Check the IMU mounting."));
    }
  }
  Serial.println(F("Set CALIBRATE_IMU to 0 and re-upload to fly."));
  while (true) {}
}
#endif