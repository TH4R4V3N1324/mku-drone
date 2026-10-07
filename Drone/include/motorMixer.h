#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "escOutput.h"

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
