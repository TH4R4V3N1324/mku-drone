#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}
    /**
     * @brief Compute the PID output
     * @details Computes the PID output based on the setpoint, measured value, and time delta
     * @param[in] setpoint The desired setpoint value
     * @param[in] measured The current measured value
     * @param[in] dt The time delta in seconds since the last computation
     * @return The computed PID output
     */
    float compute(float setpoint, float measured, float dt) {
        float error = setpoint - measured;
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        prevError = error;
        return kp * error + ki * integral + kd * derivative;
    }
private:
    float kp;
    float ki;
    float kd;
    float prevError;
    float integral;
};

#endif // PID_H