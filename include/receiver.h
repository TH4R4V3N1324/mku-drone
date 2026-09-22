#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver {
public:
    Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin);
    void init();
    void readData();
    void printData() const;
    void handlePinChangeInterrupt();
    float getThrottle() const { return throttle; }
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }

private:
    float readChannel(uint16_t pulseWidth, float minimum, float maximum) const;
    uint8_t throttlePin;
    uint8_t rollPin;
    uint8_t pitchPin;
    uint8_t yawPin;
    float throttle;
    float roll;
    float pitch;
    float yaw;
    volatile uint16_t pulseWidths[4];
    volatile unsigned long pulseStarts[4];
    volatile unsigned long lastPulseTimes[4];
    volatile uint8_t lastPortState;
};

#endif // RECEIVER_H