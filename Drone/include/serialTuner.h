#ifndef SERIAL_TUNER_H
#define SERIAL_TUNER_H

#include <Arduino.h>
#include "pid.h"

/**
 * @brief Runtime PID gain tuner over Serial — no reflash needed.
 * @details Call update() once per loop(). Type a line + Enter into your
 *          serial monitor (Teleplot's console also echoes commands sent
 *          this way — see note below on port sharing).
 *
 *          Commands:
 *            axis rr     -> select roll rate (inner loop) for tuning
 *            axis r      -> select roll angle (outer loop)
 *            axis pr     -> select pitch rate (inner loop)
 *            axis p      -> select pitch angle (outer loop)
 *            axis y      -> select yaw rate
 *            axis h      -> select hover/altitude
 *            p 1.5       -> set kp = 1.5 on the selected axis
 *            i 0.04      -> set ki = 0.04
 *            d 18.0      -> set kd = 18.0
 *            reset       -> reset() the selected axis (clears integral/derivative state)
 *            show        -> print the selected axis's current gains
 */
class SerialTuner {
public:
    /**
     * @brief Construct a new SerialTuner object
     * @param[in] rollAnglePID   Reference to the roll angle (outer) PID controller
     * @param[in] rollRatePID    Reference to the roll rate (inner) PID controller
     * @param[in] pitchAnglePID  Reference to the pitch angle (outer) PID controller
     * @param[in] pitchRatePID   Reference to the pitch rate (inner) PID controller
     * @param[in] yawPID         Reference to the yaw rate PID controller
     * @param[in] hoverPID       Reference to the hover PID controller
     */
    SerialTuner(PID &rollAnglePID, PID &rollRatePID, PID &pitchAnglePID, PID &pitchRatePID,
                PID &yawPID, PID &hoverPID)
        : rollAnglePID(rollAnglePID), rollRatePID(rollRatePID),
          pitchAnglePID(pitchAnglePID), pitchRatePID(pitchRatePID),
          yawPID(yawPID), hoverPID(hoverPID),
          active(&rollRatePID), activeName("rollrate") {}

    /**
     * @brief Get the name of the axis currently selected for tuning
     * @return The name of the active axis ("roll", "rollrate", "pitch", "pitchrate", "yaw", or "hover")
     */
    const char* getActiveAxisName() const { return activeName; }

    /**
     * @brief Call this once per loop() to process any incoming serial commands
     * @return None
     */
    void update() {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (lineBuffer.length() > 0) {
                    handleLine(lineBuffer);
                    lineBuffer = "";
                }
            } else {
                lineBuffer += c;
            }
        }
    }

private:
    PID &rollAnglePID;
    PID &rollRatePID;
    PID &pitchAnglePID;
    PID &pitchRatePID;
    PID &yawPID;
    PID &hoverPID;
    PID *active;
    const char *activeName;
    String lineBuffer;

    /**
     * @brief Handle a line of input from the serial monitor
     * @param line The line of input to process
     * @return None
     */
    void handleLine(String line) {
        line.trim();
        line.toLowerCase();

        if (line.startsWith("axis ")) {
            String axis = line.substring(5);
            axis.trim();
            if (axis == "rr") { active = &rollRatePID; activeName = "rollrate"; }
            else if (axis == "r") { active = &rollAnglePID; activeName = "roll"; }
            else if (axis == "pr") { active = &pitchRatePID; activeName = "pitchrate"; }
            else if (axis == "p") { active = &pitchAnglePID; activeName = "pitch"; }
            else if (axis == "y") { active = &yawPID; activeName = "yaw"; }
            else if (axis == "h") { active = &hoverPID; activeName = "hover"; }
            else { Serial.println(F("Unknown axis. Use: axis rr | r | pr | p | y | h")); return; }
            Serial.print(F("Active axis: "));
            Serial.println(activeName);
        }
        else if (line.startsWith("p ")) {
            active->setKp(line.substring(2).toFloat());
            printGains();
        }
        else if (line.startsWith("i ")) {
            active->setKi(line.substring(2).toFloat());
            printGains();
        }
        else if (line.startsWith("d ")) {
            active->setKd(line.substring(2).toFloat());
            printGains();
        }
        else if (line == "reset") {
            active->reset();
            Serial.print(activeName);
            Serial.println(F(" reset."));
        }
        else if (line == "show") {
            printGains();
        }
        else {
            Serial.println(F("Unknown command. Try: axis rr/r/pr/p/y/h | p <val> | i <val> | d <val> | reset | show"));
        }
    }

    /**
     * @brief Print the current gains of the active PID controller
     * @return None
     */
    void printGains() {
        Serial.print(activeName);
        Serial.print(F(" gains -> kp: "));
        Serial.print(active->getKp(), 4);
        Serial.print(F(" ki: "));
        Serial.print(active->getKi(), 4);
        Serial.print(F(" kd: "));
        Serial.println(active->getKd(), 4);
    }
};

#endif // SERIAL_TUNER_H
