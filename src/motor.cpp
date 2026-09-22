#include "motor.h"

Motor::Motor(int pwmPin) : pwmPin(pwmPin) {
    motor.attach(pwmPin);
}

void Motor::setSpeed(float speed) {
    speed = constrain(speed, 0, 1);
    int speedMapped = static_cast<int>(1000.0f + speed * 1000.0f);
    motor.writeMicroseconds(speedMapped);
}

void Motor::stop() {
    motor.writeMicroseconds(1000); // Minimum throttle for a standard ESC
}