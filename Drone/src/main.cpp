#include <Arduino.h>
#include "imu.h"
#include "motorMixer.h"
#include "motor.h"
#include "pid.h"
#include "receiver.h"
#include <Wire.h>

void run();
void test();

Motor motor1(4); // PWM pin for motor 1
Motor motor2(5); // PWM pin for motor 2
Motor motor3(6); // PWM pin for motor 3
Motor motor4(7); // PWM pin for motor 4
MotorMixer mixer(motor1, motor2, motor3, motor4);
IMU imu;

constexpr uint8_t throttlePin = 9;
constexpr uint8_t rollPin = 8;
constexpr uint8_t pitchPin = 10;
constexpr uint8_t yawPin = 11;
Receiver receiver(throttlePin, rollPin, pitchPin, yawPin);

PID rollPID(0.02, 0.0, 0.0, -0.3, 0.3); // PID controller for roll
PID pitchPID(0.02, 0.0, 0.0, -0.3, 0.3); // PID controller for pitch
PID yawPID(0.05, 0.0, 0.0, -0.2, 0.2); // PID controller for yaw
unsigned long previousLoopTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  imu.init();
  imu.calibrate();
  receiver.init();
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  delay(2000);
  previousLoopTime = micros();
}

void loop() {
  //run();
  test();
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

  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  imu.readData();
  //imu.printOrientation();
  receiver.readData();
  //receiver.printData();

  float throttle = receiver.getThrottle();
  float rollSetpoint = receiver.getRoll() * 30.0f;
  float pitchSetpoint = receiver.getPitch() * 30.0f;
  float yawRateSetpoint = receiver.getYaw() * 180.0f;

  float rollOutput = rollPID.compute(rollSetpoint, imu.getRoll(), dt);
  float pitchOutput = pitchPID.compute(pitchSetpoint, imu.getPitch(), dt);
  float yawOutput = yawPID.compute(yawRateSetpoint, imu.getGyroZ(), dt);

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
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    return;
  }

  mixer.mixMotors(throttle, roll, pitch, yaw);
}