#include "receiver.h"
#include <avr/interrupt.h>

namespace {
Receiver* activeReceiver = nullptr;

constexpr unsigned long SIGNAL_TIMEOUT_US = 100000UL; // channel is lost if no pulse for 100 ms
constexpr uint16_t MIN_VALID_PULSE_US = 900;
constexpr uint16_t MAX_VALID_PULSE_US = 2100;
constexpr float STICK_DEADBAND = 0.05f;

/**
 * @brief Zero small stick values and rescale the rest so output still starts at 0 and reaches +/-1
 */
float applyDeadband(float value) {
    const float magnitude = fabsf(value);
    if (magnitude <= STICK_DEADBAND) {
        return 0.0f;
    }
    return copysignf((magnitude - STICK_DEADBAND) / (1.0f - STICK_DEADBAND), value);
}
}

/**
 * @brief Initialize the receiver
 * @details Initializes the receiver by setting up the input pins for throttle, roll, pitch, and yaw channels
 * @param[in] throttlePin The pin for the throttle channel
 * @param[in] rollPin The pin for the roll channel
 * @param[in] pitchPin The pin for the pitch channel
 * @param[in] yawPin The pin for the yaw channel
 * @param[in] aux1Pin The pin for the auxiliary channel 1
 * @param[in] aux2Pin The pin for the auxiliary channel 2
 * @return None
 */
Receiver::Receiver(uint8_t throttlePin, uint8_t rollPin, uint8_t pitchPin, uint8_t yawPin, uint8_t aux1Pin, uint8_t aux2Pin)
    : throttlePin(throttlePin),
      rollPin(rollPin),
      pitchPin(pitchPin),
      yawPin(yawPin),
      aux1Pin(aux1Pin),
      aux2Pin(aux2Pin),
      throttle(0.0f),
      roll(0.0f),
      pitch(0.0f),
    yaw(0.0f),
    aux1(0.0f),
    aux2(0.0f),
    signalValid(false),
    pulseWidths{0, 0, 0, 0, 0, 0},
    pulseStarts{0, 0, 0, 0, 0, 0},
    lastPulseTimes{0, 0, 0, 0, 0, 0},
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

    if (aux1Pin != NOT_A_PIN) {
        pinMode(aux1Pin, INPUT);
    }

    if (aux2Pin != NOT_A_PIN) {
        pinMode(aux2Pin, INPUT);
    }

    activeReceiver = this;
    lastPortState = PINB & 0x3F;
    PCICR |= _BV(PCIE0);
    PCMSK0 = 0;
    PCMSK0 |= _BV(throttlePin - 8) | _BV(rollPin - 8) |
              _BV(pitchPin - 8) | _BV(yawPin - 8);
    if (aux1Pin != NOT_A_PIN) {
        PCMSK0 |= _BV(aux1Pin - 8);
    }
    if (aux2Pin != NOT_A_PIN) {
        PCMSK0 |= _BV(aux2Pin - 8);
    }
}

/**
 * @brief Read data from the receiver
 * @details Reads the throttle, roll, pitch, and yaw values from the receiver channels
 * @return None
 */
void Receiver::readData() {
    uint16_t pulseSnapshot[6];
    unsigned long lastPulseSnapshot[6];

    noInterrupts();
    for (uint8_t channel = 0; channel < 6; ++channel) {
        pulseSnapshot[channel] = pulseWidths[channel];
        lastPulseSnapshot[channel] = lastPulseTimes[channel];
    }
    interrupts();

    const unsigned long now = micros();

    // Normalized channel value, or false (value 0) if the channel is unused, stale, or out of range
    auto readPin = [&](uint8_t pin, float minimum, float maximum, float& value) -> bool {
        value = 0.0f;
        if (pin == NOT_A_PIN) {
            return false;
        }
        const uint8_t channel = pin - 8;
        const uint16_t pulse = pulseSnapshot[channel];
        if (now - lastPulseSnapshot[channel] > SIGNAL_TIMEOUT_US ||
            pulse < MIN_VALID_PULSE_US || pulse > MAX_VALID_PULSE_US) {
            return false;
        }
        value = readChannel(pulse, minimum, maximum);
        return true;
    };

    signalValid = readPin(throttlePin, 0.0f, 1.0f, throttle);
    readPin(rollPin, -1.0f, 1.0f, roll);
    readPin(pitchPin, -1.0f, 1.0f, pitch);
    readPin(yawPin, -1.0f, 1.0f, yaw);
    readPin(aux1Pin, 0.0f, 1.0f, aux1);
    readPin(aux2Pin, 0.0f, 1.0f, aux2);

    throttle = applyDeadband(throttle);
    roll = applyDeadband(roll);
    pitch = -applyDeadband(pitch);
    yaw = applyDeadband(yaw);
}

void Receiver::handlePinChangeInterrupt() {
    const uint8_t portState = PINB & 0x3F;
    const uint8_t changedBits = portState ^ lastPortState;
    const unsigned long currentTime = micros();

    for (uint8_t channel = 0; channel < 6; ++channel) {
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
 * @brief Convert a validated pulse width to a normalized channel value
 * @details Maps 1000-2000 us onto [minimum, maximum], clamping pulses slightly outside that range
 * @param[in] pulseWidth The captured receiver pulse width in microseconds
 * @param[in] minimum The minimum value for the channel
 * @param[in] maximum The maximum value for the channel
 * @return The normalized value of the channel
 */
float Receiver::readChannel(uint16_t pulseWidth, float minimum, float maximum) const {
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
    Serial.print(" | Yaw: "); Serial.print(yaw);
    Serial.print(" | AUX1: "); Serial.print(aux1);
    Serial.print(" | AUX2: "); Serial.println(aux2);
}
