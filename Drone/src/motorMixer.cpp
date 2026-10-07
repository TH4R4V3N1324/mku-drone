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
    const uint16_t pulses[ESC_COUNT] = {
        speedToPulse(throttle + roll + pitch + yaw), // Front Left
        speedToPulse(throttle - roll + pitch - yaw), // Front Right
        speedToPulse(throttle - roll - pitch + yaw), // Rear Right
        speedToPulse(throttle + roll - pitch - yaw), // Rear Left
    };
    esc.write(pulses);
}

/**
 * @brief Stop all motors
 * @return None
 */
void MotorMixer::stopAllMotors() {
    esc.writeAll(ESC_MIN_PULSE_US);
}

/**
 * @brief Command all motors to the same speed without mixing (used for ESC calibration)
 * @param[in] speed The speed of every motor (0.0 to 1.0)
 * @return None
 */
void MotorMixer::setAllMotors(float speed) {
    esc.writeAll(speedToPulse(speed));
}

/**
 * @brief Convert a normalized motor speed to an ESC pulse width
 * @param[in] speed The speed of the motor (clamped to 0.0 to 1.0)
 * @return Pulse width in microseconds
 */
uint16_t MotorMixer::speedToPulse(float speed) {
    speed = constrain(speed, 0.0f, 1.0f);
    return static_cast<uint16_t>(ESC_MIN_PULSE_US + speed * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US));
}
