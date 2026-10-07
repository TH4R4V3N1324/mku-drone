#ifndef ESC_OUTPUT_H
#define ESC_OUTPUT_H

#include <Arduino.h>

constexpr uint8_t ESC_COUNT = 4;
constexpr uint16_t ESC_MIN_PULSE_US = 1000;
constexpr uint16_t ESC_MAX_PULSE_US = 2000;

// ESC update / control loop rate. Standard PWM ESCs accept up to ~400-490 Hz.
constexpr uint16_t ESC_UPDATE_RATE_HZ = 250;

/**
 * @brief Timer1-driven PWM output for four ESCs
 * @details All four pins go high together at the start of each cycle and a
 * compare interrupt drops each one at its pulse width, so nothing busy-waits.
 * write() hands over all four widths at once so every cycle is consistent.
 * If write() is not called for 200 ms the outputs fall back to minimum throttle.
 *
 * Uses Timer1, so it cannot be combined with the Servo library or analogWrite
 * on pins 9/10. Only one instance may exist. Pins must be on PORTD (Arduino
 * pins 2-7; 0/1 are Serial).
 */
class EscOutput {
public:
    EscOutput() : lastCycle(0) {}
    void begin(const uint8_t (&pins)[ESC_COUNT]);
    void write(const uint16_t (&pulseUs)[ESC_COUNT]);
    void writeAll(uint16_t pulseUs);
    void waitForNextCycle();
private:
    uint8_t portMasks[ESC_COUNT];
    uint8_t lastCycle;
};

#endif // ESC_OUTPUT_H
