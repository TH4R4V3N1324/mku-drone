#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    /**
     * @brief Construct a new PID object
     * @param[in] kp         Proportional gain
     * @param[in] ki         Integral gain
     * @param[in] kd         Derivative gain
     * @param[in] outputMin  Minimum output value (clamped)
     * @param[in] outputMax  Maximum output value (clamped)
     * @param[in] dFilterHz  Low-pass cutoff for the derivative term only (0 = unfiltered)
     */
    PID(float kp, float ki, float kd, float outputMin, float outputMax, float dFilterHz = 0.0f) :
        kp(kp), ki(ki), kd(kd),
        outputMin(outputMin), outputMax(outputMax),
        dFilterTau(dFilterHz > 0.0f ? 1.0f / (2.0f * PI * dFilterHz) : 0.0f),
        prevMeasured(0), integral(0), filteredDerivative(0), firstRun(true) {
        updateIntegralLimits();
    }

    /**
     * @brief Compute the PID output
     * @param[in] setpoint  Desired value
     * @param[in] measured  Current measured value
     * @param[in] dt        Time delta in seconds since the last call (must be > 0)
     * @return Clamped PID output in [outputMin, outputMax]
     */
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) {
            return 0.0f; // bad timestep — don't poison integral/derivative
        }

        float error = setpoint - measured;

        // Float division is slow on the AVR (~30 us), so the clamp limits and filter constant
        // are precomputed and the derivative path uses at most one division.

        //Derivative on measurement (avoids setpoint-change kick), optionally low-passed; skipped when kd is 0
        float derivative = 0.0f;
        if (!firstRun && kd != 0.0f) {
            const float delta = measured - prevMeasured;
            if (dFilterTau > 0.0f) {
                // Same as filtered += dt/(tau+dt) * (-delta/dt - filtered), with one division instead of two
                filteredDerivative += (-delta - dt * filteredDerivative) / (dFilterTau + dt);
                derivative = filteredDerivative;
            } else {
                derivative = -delta / dt;
            }
        }
        prevMeasured = measured;
        firstRun = false;

        const float pdTerm = kp * error + kd * derivative;

        //Integral with anti-windup: clamped, and frozen while the output is already saturated in the
        //direction the error pushes, so it can't build up during a long saturated input and then keep
        //driving after the input ends. It still unwinds normally when the error reverses.
        if (ki > 0.0f) {
            const float candidate = constrain(integral + error * dt, integralMin, integralMax);
            const float unclamped = pdTerm + ki * candidate;
            const bool pushingPastMax = unclamped > outputMax && error > 0.0f;
            const bool pushingPastMin = unclamped < outputMin && error < 0.0f;
            if (!pushingPastMax && !pushingPastMin) {
                integral = candidate;
            }
        } else {
            integral = 0.0f;
        }

        float output = pdTerm + ki * integral;
        return constrain(output, outputMin, outputMax);
    }

    /**
     * @brief Reset internal state (call when re-arming / switching modes)
     */
    void reset() {
        integral = 0.0f;
        prevMeasured = 0.0f;
        filteredDerivative = 0.0f;
        firstRun = true;
    }

    /**
     * @brief Clear only the integral, keeping the derivative history (call every loop to hold I at zero)
     */
    void resetIntegral() { integral = 0.0f; }

    void setKp(float newKp) { kp = newKp; }
    void setKi(float newKi) { ki = newKi; updateIntegralLimits(); }
    void setKd(float newKd) { kd = newKd; filteredDerivative = 0.0f; } // the filter isn't updated while kd is 0
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    float getIntegralTerm() const { return ki * integral; } // the I contribution to the output

private:
    /**
     * @brief Recompute the integral clamp so ki * integral stays within the output range
     */
    void updateIntegralLimits() {
        integralMin = ki > 0.0f ? outputMin / ki : 0.0f;
        integralMax = ki > 0.0f ? outputMax / ki : 0.0f;
    }

    float kp, ki, kd;
    float outputMin, outputMax;
    float integralMin, integralMax;
    float dFilterTau; // s, derivative low-pass time constant (0 = unfiltered)
    float prevMeasured;
    float integral;
    float filteredDerivative;
    bool firstRun;
};

#endif // PID_H
