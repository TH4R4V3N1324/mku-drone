#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "motor.h"

class MotorMixer {
public:
    MotorMixer(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4)
        : motor1(motor1), motor2(motor2), motor3(motor3), motor4(motor4) {}
    void mixMotors(float throttle, float roll, float pitch, float yaw);
private:
    Motor& motor1;
    Motor& motor2;
    Motor& motor3;
    Motor& motor4;
};

#endif // MOTOR_MIXER_H