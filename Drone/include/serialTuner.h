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
 *            axis r      -> select roll for tuning
 *            axis p      -> select pitch
 *            axis y      -> select yaw
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
     * @param[in] rollPID   Reference to the roll PID controller
     * @param[in] pitchPID  Reference to the pitch PID controller
     * @param[in] yawPID    Reference to the yaw PID controller
     */
    SerialTuner(PID &rollPID, PID &pitchPID, PID &yawPID)
        : rollPID(rollPID), pitchPID(pitchPID), yawPID(yawPID),
          active(&rollPID), activeName("roll") {}

    /**
     * @brief Get the name of the axis currently selected for tuning
     * @return The name of the active axis ("roll", "pitch", or "yaw")
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
    PID &rollPID;
    PID &pitchPID;
    PID &yawPID;
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
            char axis = line.charAt(5);
            if (axis == 'r') { active = &rollPID; activeName = "roll"; }
            else if (axis == 'p') { active = &pitchPID; activeName = "pitch"; }
            else if (axis == 'y') { active = &yawPID; activeName = "yaw"; }
            else { Serial.println(F("Unknown axis. Use: axis r | axis p | axis y")); return; }
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
            Serial.println(F("Unknown command. Try: axis r/p/y | p <val> | i <val> | d <val> | reset | show"));
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
