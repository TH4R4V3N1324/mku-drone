#include "escOutput.h"

namespace {
constexpr uint16_t TICKS_PER_US = 2;                                    // Timer1 at 16 MHz / 8
constexpr uint16_t PERIOD_TICKS = (F_CPU / 8UL) / ESC_UPDATE_RATE_HZ;
constexpr uint16_t MIN_PULSE_TICKS = ESC_MIN_PULSE_US * TICKS_PER_US;
constexpr uint16_t EDGE_MARGIN_TICKS = 10;                              // edges closer than 5 us are busy-waited in the ISR
constexpr uint8_t WATCHDOG_CYCLES = ESC_UPDATE_RATE_HZ / 5;             // 200 ms without a new command -> min throttle

static_assert(F_CPU == 16000000UL, "Timer1 tick math assumes a 16 MHz clock");
static_assert(PERIOD_TICKS >= (ESC_MAX_PULSE_US + 200) * TICKS_PER_US, "ESC rate too high for a 2 ms max pulse");

// Written by write() (with interrupts off), read at the start of each cycle.
volatile uint16_t pendingTicks[ESC_COUNT];
volatile uint8_t pendingMasks[ESC_COUNT];
volatile uint8_t cyclesSinceUpdate = WATCHDOG_CYCLES;
volatile uint8_t cycleCount = 0;

// Only touched inside the ISRs (and before the timer starts).
uint8_t escPortMask = 0;
uint16_t activeEdges[ESC_COUNT];
uint8_t activeMasks[ESC_COUNT];
uint8_t nextEdge = ESC_COUNT;
}

/**
 * @brief Start of an ESC cycle: raise all ESC pins and schedule the falling edges
 */
ISR(TIMER1_COMPA_vect) {
    PORTD |= escPortMask;
    uint16_t rise = TCNT1;
    if (rise >= PERIOD_TICKS / 2) {
        rise = 0; // read just before the counter wrapped
    }

    const bool expired = cyclesSinceUpdate >= WATCHDOG_CYCLES;
    if (!expired) {
        ++cyclesSinceUpdate;
    }

    for (uint8_t i = 0; i < ESC_COUNT; ++i) {
        activeEdges[i] = rise + (expired ? MIN_PULSE_TICKS : pendingTicks[i]);
        activeMasks[i] = pendingMasks[i];
    }

    nextEdge = 0;
    OCR1B = activeEdges[0];
    ++cycleCount;
}

/**
 * @brief Drop each ESC pin once its pulse width has elapsed (edges are sorted)
 */
ISR(TIMER1_COMPB_vect) {
    while (nextEdge < ESC_COUNT) {
        const uint16_t edge = activeEdges[nextEdge];
        if (edge > TCNT1 + EDGE_MARGIN_TICKS) {
            OCR1B = edge;
            return;
        }
        while (TCNT1 < edge) {}
        PORTD &= ~activeMasks[nextEdge];
        ++nextEdge;
    }
}

/**
 * @brief Configure the ESC pins and start the Timer1 output at minimum throttle
 * @details Pins that are not on PORTD get an empty mask and are never driven.
 * @param[in] pins The four ESC signal pins
 * @return None
 */
void EscOutput::begin(const uint8_t (&pins)[ESC_COUNT]) {
    uint8_t allMasks = 0;
    for (uint8_t i = 0; i < ESC_COUNT; ++i) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
        portMasks[i] = (portOutputRegister(digitalPinToPort(pins[i])) == &PORTD)
            ? digitalPinToBitMask(pins[i])
            : 0;
        allMasks |= portMasks[i];
    }
    escPortMask = allMasks;
    writeAll(ESC_MIN_PULSE_US);

    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = PERIOD_TICKS - 1;           // cycle length (CTC TOP)
    OCR1B = PERIOD_TICKS - 1;
    TIFR1 = _BV(OCF1A) | _BV(OCF1B);
    TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
    TCCR1B = _BV(WGM12) | _BV(CS11);    // CTC mode, prescaler 8
    interrupts();
}

/**
 * @brief Set the pulse widths for the next output cycle
 * @details All four widths are handed to the ISR together, so a cycle never
 * mixes old and new values. Widths are clamped to the ESC range.
 * @param[in] pulseUs Pulse width per ESC in microseconds, in begin() pin order
 * @return None
 */
void EscOutput::write(const uint16_t (&pulseUs)[ESC_COUNT]) {
    uint16_t ticks[ESC_COUNT];
    uint8_t masks[ESC_COUNT];

    // Insertion sort so the ISR can drop the pins in order
    for (uint8_t i = 0; i < ESC_COUNT; ++i) {
        const uint16_t t = constrain(pulseUs[i], ESC_MIN_PULSE_US, ESC_MAX_PULSE_US) * TICKS_PER_US;
        uint8_t j = i;
        while (j > 0 && ticks[j - 1] > t) {
            ticks[j] = ticks[j - 1];
            masks[j] = masks[j - 1];
            --j;
        }
        ticks[j] = t;
        masks[j] = portMasks[i];
    }

    noInterrupts();
    for (uint8_t i = 0; i < ESC_COUNT; ++i) {
        pendingTicks[i] = ticks[i];
        pendingMasks[i] = masks[i];
    }
    cyclesSinceUpdate = 0;
    interrupts();
}

/**
 * @brief Set every ESC to the same pulse width for the next output cycle
 * @param[in] pulseUs Pulse width in microseconds
 * @return None
 */
void EscOutput::writeAll(uint16_t pulseUs) {
    const uint16_t pulses[ESC_COUNT] = {pulseUs, pulseUs, pulseUs, pulseUs};
    write(pulses);
}

/**
 * @brief Block until the next ESC output cycle starts
 * @details Paces the control loop at ESC_UPDATE_RATE_HZ. Returns immediately if
 * a cycle already started since the last call (i.e. the loop overran).
 * @return None
 */
void EscOutput::waitForNextCycle() {
    while (cycleCount == lastCycle) {}
    lastCycle = cycleCount;
}
