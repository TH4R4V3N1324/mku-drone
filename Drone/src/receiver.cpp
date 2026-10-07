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
    : pins{throttlePin, rollPin, pitchPin, yawPin, aux1Pin, aux2Pin},
      channelPorts{0, 0, 0, 0, 0, 0},
      channelMasks{0, 0, 0, 0, 0, 0},
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
    lastPortStates{0, 0, 0} {}

/**
 * @brief Initialize the receiver
 * @details Initializes the receiver by setting up the input pins for throttle, roll, pitch, and yaw channels
 * @return None
 */
void Receiver::init() {
    activeReceiver = this;
    lastPortStates[0] = PINB;
    lastPortStates[1] = PINC;
    lastPortStates[2] = PIND;

    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        const uint8_t pin = pins[channel];
        if (pin == NOT_A_PIN) {
            continue;
        }
        pinMode(pin, INPUT);
        channelPorts[channel] = digitalPinToPCICRbit(pin);
        channelMasks[channel] = _BV(digitalPinToPCMSKbit(pin));
        *digitalPinToPCMSK(pin) |= channelMasks[channel];
        PCICR |= _BV(channelPorts[channel]);
    }
}

/**
 * @brief Read data from the receiver
 * @details Reads the throttle, roll, pitch, and yaw values from the receiver channels
 * @return None
 */
void Receiver::readData() {
    uint16_t pulseSnapshot[CHANNEL_COUNT];
    unsigned long lastPulseSnapshot[CHANNEL_COUNT];

    noInterrupts();
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        pulseSnapshot[channel] = pulseWidths[channel];
        lastPulseSnapshot[channel] = lastPulseTimes[channel];
    }
    interrupts();

    const unsigned long now = micros();

    // Normalized channel value, or false (value 0) if the channel is unused, stale, or out of range
    auto readPin = [&](uint8_t channel, float minimum, float maximum, float& value) -> bool {
        value = 0.0f;
        if (channelMasks[channel] == 0) {
            return false;
        }
        const uint16_t pulse = pulseSnapshot[channel];
        if (now - lastPulseSnapshot[channel] > SIGNAL_TIMEOUT_US ||
            pulse < MIN_VALID_PULSE_US || pulse > MAX_VALID_PULSE_US) {
            return false;
        }
        value = readChannel(pulse, minimum, maximum);
        return true;
    };

    signalValid = readPin(THROTTLE, 0.0f, 1.0f, throttle);
    readPin(ROLL, -1.0f, 1.0f, roll);
    readPin(PITCH, -1.0f, 1.0f, pitch);
    readPin(YAW, -1.0f, 1.0f, yaw);
    readPin(AUX1, 0.0f, 1.0f, aux1);
    readPin(AUX2, 0.0f, 1.0f, aux2);

    throttle = applyDeadband(throttle);
    roll = applyDeadband(roll);
    pitch = -applyDeadband(pitch);
    yaw = applyDeadband(yaw);
}

/**
 * @brief Time pulse edges on one port
 * @param[in] port Pin-change group that fired (0 = PORTB, 1 = PORTC, 2 = PORTD)
 * @param[in] portState Current PINx value of that port
 */
void Receiver::handlePinChangeInterrupt(uint8_t port, uint8_t portState) {
    const uint8_t changedBits = portState ^ lastPortStates[port];
    const unsigned long currentTime = micros();

    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        const uint8_t channelMask = channelMasks[channel];
        if (channelPorts[channel] != port || (changedBits & channelMask) == 0) {
            continue;
        }

        if (portState & channelMask) {
            pulseStarts[channel] = currentTime;
        } else {
            pulseWidths[channel] = static_cast<uint16_t>(currentTime - pulseStarts[channel]);
            lastPulseTimes[channel] = currentTime;
        }
    }

    lastPortStates[port] = portState;
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
        activeReceiver->handlePinChangeInterrupt(0, PINB);
    }
}

ISR(PCINT1_vect) {
    if (activeReceiver != nullptr) {
        activeReceiver->handlePinChangeInterrupt(1, PINC);
    }
}

ISR(PCINT2_vect) {
    if (activeReceiver != nullptr) {
        activeReceiver->handlePinChangeInterrupt(2, PIND);
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
