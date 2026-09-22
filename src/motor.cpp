#include "motor.h"

/**
 * @brief Initialize the motor
 * @details Initializes the motor by attaching it to the specified PWM pin
 * @param[in] pwmPin The PWM pin to which the motor is attached
 * @return None
 */
Motor::Motor(int pwmPin) : pwmPin(pwmPin) {
}

/**
 * @brief Begin the motor operation
 * @details Attaches the motor to the specified PWM pin and stops it initially
 * @return None
 */
void Motor::begin() {
    motor.attach(pwmPin);
    stop();
}

/**
 * @brief Set the speed of the motor
 * @details Sets the speed of the motor by writing a pulse width to the attached servo
 * @param[in] speed The speed of the motor (0.0 to 1.0)
 * @return None
 */
void Motor::setSpeed(float speed) {
    speed = constrain(speed, 0, 1);
    int speedMapped = static_cast<int>(1000.0f + speed * 1000.0f);
    motor.writeMicroseconds(speedMapped);
}

/**
 * @brief Stop the motor
 * @details Stops the motor by setting its speed to zero
 * @return None
 */
void Motor::stop() {
    motor.writeMicroseconds(1000); // Minimum throttle for a standard ESC
}