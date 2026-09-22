#include "receiver.h"

/**
 * @brief Initialize the receiver
 * @details Initializes the receiver by setting up the input pins for throttle, roll, pitch, and yaw channels
 * @param[in] throttlePin The pin for the throttle channel
 * @param[in] rollPin The pin for the roll channel
 * @param[in] pitchPin The pin for the pitch channel
 * @param[in] yawPin The pin for the yaw channel
 * @return None
 */
Receiver::Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin)
    : throttlePin(throttlePin),
      rollPin(rollPin),
      pitchPin(pitchPin),
      yawPin(yawPin),
      throttle(0.0f),
      roll(0.0f),
      pitch(0.0f),
      yaw(0.0f) {}

/**
 * @brief Initialize the receiver
 * @details Initializes the receiver by setting up the input pins for throttle, roll, pitch, and yaw channels
 * @return None
 */
void Receiver::init() {
    pinMode(throttlePin, INPUT);
    pinMode(rollPin, INPUT);
    pinMode(pitchPin, INPUT);
    pinMode(yawPin, INPUT);
}

/**
 * @brief Read data from the receiver
 * @details Reads the throttle, roll, pitch, and yaw values from the receiver channels
 * @return None
 */
void Receiver::readData() {
    float newThrottle = readChannel(throttlePin, 0.0f, 1.0f);
    float newRoll = readChannel(rollPin, -1.0f, 1.0f);
    float newPitch = readChannel(pitchPin, -1.0f, 1.0f);
    float newYaw = readChannel(yawPin, -1.0f, 1.0f);

    if (newThrottle >= 0.0f) throttle = newThrottle;
    if (newRoll >= -1.0f) roll = newRoll;
    if (newPitch >= -1.0f) pitch = newPitch;
    if (newYaw >= -1.0f) yaw = newYaw;
}

/**
 * @brief Read a single channel from the receiver
 * @details Reads the pulse width of a single channel and converts it to a normalized value
 * @param[in] pin The pin to read from
 * @param[in] minimum The minimum value for the channel
 * @param[in] maximum The maximum value for the channel
 * @return The normalized value of the channel, or minimum - 1.0f if the pulse width is out of range
 */
float Receiver::readChannel(uint8_t pin, float minimum, float maximum) const {
    unsigned long pulseWidth = pulseIn(pin, HIGH, 25000UL);

    if (pulseWidth < 900UL || pulseWidth > 2100UL) {
        return minimum - 1.0f;
    }

    float value = (pulseWidth - 1000.0f) / 1000.0f;
    value = constrain(value, 0.0f, 1.0f);
    return minimum + value * (maximum - minimum);
}

/**
 * @brief Print the data from the receiver
 * @details Prints the throttle, roll, pitch, and yaw values to the serial monitor
 * @return None
 */
void Receiver::printData() const {
    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print(" | Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);
}
