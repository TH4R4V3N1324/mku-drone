#include "motorMixer.h"

/**
 * @brief Mix the control signals to determine the motor speeds
 * @details Mixes the throttle, roll, pitch, and yaw signals to determine the speed of each motor
 * @param[in] throttle The throttle input (0.0 to 1.0)
 * @param[in] roll The roll input (0.0 to 1.0)
 * @param[in] pitch The pitch input (0.0 to 1.0)
 * @param[in] yaw The yaw input (0.0 to 1.0)
 * @return None
 */
void MotorMixer::mixMotors(float throttle, float roll, float pitch, float yaw) {
    // Simple mixing algorithm for a quadcopter in X configuration
    float motor1Speed = throttle + roll + pitch + yaw; // Front Left
    float motor2Speed = throttle - roll + pitch - yaw; // Front Right
    float motor3Speed = throttle - roll - pitch + yaw; // Rear Right
    float motor4Speed = throttle + roll - pitch - yaw; // Rear Left

    // Keep mixer outputs in the normalized Motor speed range (0-1).
    motor1Speed = constrain(motor1Speed, 0.0f, 1.0f);
    motor2Speed = constrain(motor2Speed, 0.0f, 1.0f);
    motor3Speed = constrain(motor3Speed, 0.0f, 1.0f);
    motor4Speed = constrain(motor4Speed, 0.0f, 1.0f);

    // Set motor speeds
    motor1.setSpeed(motor1Speed);
    motor2.setSpeed(motor2Speed);
    motor3.setSpeed(motor3Speed);
    motor4.setSpeed(motor4Speed);
}