#include "motor.h"

Motor::Motor(int pwmPin) : pwmPin(pwmPin) {
    motor.attach(pwmPin);
}

void Motor::setSpeed(float speed) {
    speed = constrain(speed, 0, 1);
    int speedMapped = map(speed, 0, 1, 1000, 2000);
    motor.writeMicroseconds(speedMapped);
}

void Motor::stop() {
    motor.writeMicroseconds(1000); // Minimum throttle for a standard ESC
}