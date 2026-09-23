#ifndef PID_H
#define PID_H

class PID {
public:
    /**
     * @brief Construct a new PID object
     * @param[in] kp         Proportional gain
     * @param[in] ki         Integral gain
     * @param[in] kd         Derivative gain
     * @param[in] outputMin  Minimum output value (clamped)
     * @param[in] outputMax  Maximum output value (clamped)
     */
    PID(float kp, float ki, float kd, float outputMin, float outputMax) : 
        kp(kp), ki(ki), kd(kd),
        outputMin(outputMin), outputMax(outputMax),
        prevMeasured(0), integral(0), firstRun(true) {}

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
 
        //Integral with anti-windup clamp
        integral += error * dt;
        if (ki > 0.0f) {
            float integralLimit = (outputMax - outputMin) / ki;
            integral = constrain(integral, -integralLimit, integralLimit);
        } else {
            integral = 0.0f;
        }
 
        //Derivative on measurement (avoids setpoint-change kick)
        float derivative = 0.0f;
        if (!firstRun) {
            derivative = -(measured - prevMeasured) / dt;
        }
        prevMeasured = measured;
        firstRun = false;
 
        float output = kp * error + ki * integral + kd * derivative;
        return constrain(output, outputMin, outputMax);
    }
 
    /**
     * @brief Reset internal state (call when re-arming / switching modes)
     */
    void reset() {
        integral = 0.0f;
        prevMeasured = 0.0f;
        firstRun = true;
    }

    void setKp(float newKp) { kp = newKp; }
    void setKi(float newKi) { ki = newKi; }
    void setKd(float newKd) { kd = newKd; }
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }

private:
    float kp, ki, kd;
    float outputMin, outputMax;
    float prevMeasured; 
    float integral;
    bool firstRun = true;
};

#endif // PID_H