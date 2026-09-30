#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver {
public:
    Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin, uint8_t aux1Pin = NOT_A_PIN, uint8_t aux2Pin = NOT_A_PIN);
    void init();
    void readData();
    void printData() const;
    void handlePinChangeInterrupt();
    float getThrottle() const { return throttle; }
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getAux1() const { return aux1; }
    float getAux2() const { return aux2; }

private:
    float readChannel(uint16_t pulseWidth, float minimum, float maximum) const;
    uint8_t throttlePin;
    uint8_t rollPin;
    uint8_t pitchPin;
    uint8_t yawPin;
    uint8_t aux1Pin;
    uint8_t aux2Pin;
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float aux1;
    float aux2;
    volatile uint16_t pulseWidths[6];
    volatile unsigned long pulseStarts[6];
    volatile unsigned long lastPulseTimes[6];
    volatile uint8_t lastPortState;
};

#endif // RECEIVER_H