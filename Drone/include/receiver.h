#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

/**
 * @brief PWM receiver decoded with pin-change interrupts
 * @details Channel pins can be any pin except 0/1 (Serial), spread across PORTB, PORTC and PORTD.
 * The pins must not otherwise change state (e.g. no outputs on the same pins). Lost or
 * out-of-range channels read as 0 (throttle off, sticks centred, switches low).
 */
class Receiver {
public:
    Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin, uint8_t aux1Pin = NOT_A_PIN, uint8_t aux2Pin = NOT_A_PIN);
    void init();
    void readData();
    void printData() const;
    void handlePinChangeInterrupt(uint8_t port, uint8_t portState);
    float getThrottle() const { return throttle; }
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getAux1() const { return aux1; }
    float getAux2() const { return aux2; }
    /**
    * @brief Whether a valid throttle pulse arrived within the last 100 ms
    * @return True if the transmitter link is live
     */
    bool hasSignal() const { return signalValid; }

private:
    static constexpr uint8_t CHANNEL_COUNT = 6;
    static constexpr uint8_t PORT_COUNT = 3; // pin-change groups: 0 = PORTB, 1 = PORTC, 2 = PORTD
    enum Channel : uint8_t { THROTTLE, ROLL, PITCH, YAW, AUX1, AUX2 };
    float readChannel(uint16_t pulseWidth, float minimum, float maximum) const;
    uint8_t pins[CHANNEL_COUNT];
    uint8_t channelPorts[CHANNEL_COUNT]; // pin-change group of each channel
    uint8_t channelMasks[CHANNEL_COUNT]; // bit within that port, 0 if the channel is unused
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float aux1;
    float aux2;
    bool signalValid;
    volatile uint16_t pulseWidths[CHANNEL_COUNT];
    volatile unsigned long pulseStarts[CHANNEL_COUNT];
    volatile unsigned long lastPulseTimes[CHANNEL_COUNT];
    volatile uint8_t lastPortStates[PORT_COUNT];
};

#endif // RECEIVER_H