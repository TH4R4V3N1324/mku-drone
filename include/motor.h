#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>

class Motor {
public:
    Motor(int pwmPin);
    void setSpeed(float speed);
    void stop();

private:
    Servo motor;
    int pwmPin;
};

#endif // MOTOR_H