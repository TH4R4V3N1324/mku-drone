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
Receiver receiver(10, 8, 9, 11); // Pins for throttle, roll, pitch, and yaw channels

PID rollPID(1.3, 0.04, 18.0); // PID controller for roll
PID pitchPID(1.3, 0.04, 18.0); // PID controller for pitch
PID yawPID(4.0, 0.02, 0.0); // PID controller for yaw
unsigned long previousLoopTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();
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

void test() {
  receiver.readData();
  receiver.printData();

  float throttle = receiver.getThrottle();
  motor1.setSpeed(throttle);
  motor2.setSpeed(throttle);
  motor3.setSpeed(throttle);
  motor4.setSpeed(throttle);
}