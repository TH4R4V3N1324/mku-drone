#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "escOutput.h"

// Minimum speed for every motor while mixing (armed), so a large correction at low
// throttle can't stop a motor mid-air. Must be at or above the speed where your ESCs
// reliably spin the motors; stopAllMotors() still commands a full stop.
constexpr float MOTOR_IDLE_SPEED = 0.05f;

/**
 * @brief Quad-X mixer: turns throttle/roll/pitch/yaw into four ESC commands
 * @details ESC order is front left, front right, rear right, rear left,
 * matching the pin order passed to EscOutput::begin().
 */
class MotorMixer {
public:
    explicit MotorMixer(EscOutput& esc) : esc(esc) {}
    void mixMotors(float throttle, float roll, float pitch, float yaw);
    void stopAllMotors();
    void setAllMotors(float speed);
private:
    static uint16_t speedToPulse(float speed);
    EscOutput& esc;
};

#endif // MOTOR_MIXER_H
