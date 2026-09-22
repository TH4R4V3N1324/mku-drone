#include "motorMixer.h"

void MotorMixer::mixMotors(float throttle, float roll, float pitch, float yaw) {
    // Simple mixing algorithm for a quadcopter in X configuration
    float motor1Speed = throttle + roll + pitch + yaw; // Front Left
    float motor2Speed = throttle - roll + pitch - yaw; // Front Right
    float motor3Speed = throttle - roll - pitch + yaw; // Rear Right
    float motor4Speed = throttle + roll - pitch - yaw; // Rear Left

    // Keep mixer outputs in the Motor speed range (0-100).
    motor1Speed = constrain(motor1Speed, 0.0f, 100.0f);
    motor2Speed = constrain(motor2Speed, 0.0f, 100.0f);
    motor3Speed = constrain(motor3Speed, 0.0f, 100.0f);
    motor4Speed = constrain(motor4Speed, 0.0f, 100.0f);

    // Set motor speeds
    motor1.setSpeed(static_cast<int>(motor1Speed));
    motor2.setSpeed(static_cast<int>(motor2Speed));
    motor3.setSpeed(static_cast<int>(motor3Speed));
    motor4.setSpeed(static_cast<int>(motor4Speed));
}