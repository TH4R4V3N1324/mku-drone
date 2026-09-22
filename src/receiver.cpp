#include "receiver.h"

Receiver::Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin)
    : throttlePin(throttlePin),
      rollPin(rollPin),
      pitchPin(pitchPin),
      yawPin(yawPin),
      throttle(0.0f),
      roll(0.0f),
      pitch(0.0f),
      yaw(0.0f) {}

void Receiver::init() {
    pinMode(throttlePin, INPUT);
    pinMode(rollPin, INPUT);
    pinMode(pitchPin, INPUT);
    pinMode(yawPin, INPUT);
}

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

float Receiver::readChannel(uint8_t pin, float minimum, float maximum) const {
    unsigned long pulseWidth = pulseIn(pin, HIGH, 25000UL);

    if (pulseWidth < 900UL || pulseWidth > 2100UL) {
        return minimum - 1.0f;
    }

    float value = (pulseWidth - 1000.0f) / 1000.0f;
    value = constrain(value, 0.0f, 1.0f);
    return minimum + value * (maximum - minimum);
}

void Receiver::printData() const {
    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print(" | Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);
}
