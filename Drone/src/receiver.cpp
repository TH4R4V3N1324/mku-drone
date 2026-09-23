#include "receiver.h"
#include <avr/interrupt.h>

namespace {
Receiver* activeReceiver = nullptr;
}

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
            yaw(0.0f),
            pulseWidths{0, 0, 0, 0},
            pulseStarts{0, 0, 0, 0},
            lastPulseTimes{0, 0, 0, 0},
            lastPortState(0) {}

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

    activeReceiver = this;
    lastPortState = PINB & 0x0F;
    PCICR |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
}

/**
 * @brief Read data from the receiver
 * @details Reads the throttle, roll, pitch, and yaw values from the receiver channels
 * @return None
 */
void Receiver::readData() {
    uint16_t pulseSnapshot[4];
    unsigned long lastPulseSnapshot[4];

    noInterrupts();
    for (uint8_t channel = 0; channel < 4; ++channel) {
        pulseSnapshot[channel] = pulseWidths[channel];
        lastPulseSnapshot[channel] = lastPulseTimes[channel];
    }
    interrupts();

    const unsigned long now = micros();
    const uint8_t throttleChannel = throttlePin - 8;
    const uint8_t rollChannel = rollPin - 8;
    const uint8_t pitchChannel = pitchPin - 8;
    const uint8_t yawChannel = yawPin - 8;

    const bool throttleValid = now - lastPulseSnapshot[throttleChannel] <= 100000UL &&
                               pulseSnapshot[throttleChannel] >= 900U &&
                               pulseSnapshot[throttleChannel] <= 2100U;
    const bool rollValid = now - lastPulseSnapshot[rollChannel] <= 100000UL &&
                           pulseSnapshot[rollChannel] >= 900U &&
                           pulseSnapshot[rollChannel] <= 2100U;
    const bool pitchValid = now - lastPulseSnapshot[pitchChannel] <= 100000UL &&
                            pulseSnapshot[pitchChannel] >= 900U &&
                            pulseSnapshot[pitchChannel] <= 2100U;
    const bool yawValid = now - lastPulseSnapshot[yawChannel] <= 100000UL &&
                          pulseSnapshot[yawChannel] >= 900U &&
                          pulseSnapshot[yawChannel] <= 2100U;

    throttle = throttleValid ? readChannel(pulseSnapshot[throttleChannel], 0.0f, 1.0f) : 0.0f;
    roll = rollValid ? readChannel(pulseSnapshot[rollChannel], -1.0f, 1.0f) : 0.0f;
    pitch = pitchValid ? readChannel(pulseSnapshot[pitchChannel], -1.0f, 1.0f) : 0.0f;
    yaw = yawValid ? readChannel(pulseSnapshot[yawChannel], -1.0f, 1.0f) : 0.0f;
}

void Receiver::handlePinChangeInterrupt() {
    const uint8_t portState = PINB & 0x0F;
    const uint8_t changedBits = portState ^ lastPortState;
    const unsigned long currentTime = micros();

    for (uint8_t channel = 0; channel < 4; ++channel) {
        const uint8_t channelMask = _BV(channel);
        if ((changedBits & channelMask) == 0) {
            continue;
        }

        if (portState & channelMask) {
            pulseStarts[channel] = currentTime;
        } else {
            pulseWidths[channel] = static_cast<uint16_t>(currentTime - pulseStarts[channel]);
            lastPulseTimes[channel] = currentTime;
        }
    }

    lastPortState = portState;
}

/**
 * @brief Read a single channel from the receiver
 * @details Reads the pulse width of a single channel and converts it to a normalized value
 * @param[in] pulseWidth The captured receiver pulse width in microseconds
 * @param[in] minimum The minimum value for the channel
 * @param[in] maximum The maximum value for the channel
 * @return The normalized value of the channel, or minimum - 1.0f if the pulse width is out of range
 */
float Receiver::readChannel(uint16_t pulseWidth, float minimum, float maximum) const {
    if (pulseWidth < 900UL || pulseWidth > 2100UL) {
        return minimum - 1.0f;
    }

    float value = (pulseWidth - 1000.0f) / 1000.0f;
    value = constrain(value, 0.0f, 1.0f);
    return minimum + value * (maximum - minimum);
}

ISR(PCINT0_vect) {
    if (activeReceiver != nullptr) {
        activeReceiver->handlePinChangeInterrupt();
    }
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
