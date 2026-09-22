#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver {
public:
    Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin);
    void init();
    void readData();
    void printData() const;
    float getThrottle() const { return throttle; }
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }

private:
    float readChannel(uint8_t pin, float minimum, float maximum) const;
    uint8_t throttlePin;
    uint8_t rollPin;
    uint8_t pitchPin;
    uint8_t yawPin;
    float throttle;
    float roll;
    float pitch;
    float yaw;
};

#endif // RECEIVER_H