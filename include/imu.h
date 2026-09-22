#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temperature;
};

const int IMU_ADDRESS = 0x68;

class IMU {
public:
    IMU()
        : data{},
          lastReadTime(0),
          pitch(0.0f),
          roll(0.0f),
          yaw(0.0f),
          pitchOffset(0.0f),
          rollOffset(0.0f),
          yawOffset(0.0f) {}
    void init();
    void calibrate();
    void readData();
    void printData();
    void printOrientation();
    float getAccelX() const {return data.accelX;}
    float getAccelY() const {return data.accelY;}
    float getAccelZ() const {return data.accelZ;}
    float getGyroX() const {return data.gyroX;}
    float getGyroY() const {return data.gyroY;}
    float getGyroZ() const {return data.gyroZ;}
    float getTemperature() const {return data.temperature;}
    float getPitch() const {return pitch;}
    float getRoll() const {return roll;}
    float getYaw() const {return yaw;}
private:
    void calculateOrientation();
    IMUData data;
    unsigned long lastReadTime;
    const float ACCEL_SCALE = 16384.0; // Scale factor for accelerometer
    const float GYRO_SCALE = 131.0; // Scale factor for gyroscope
    const float TEMP_SCALE = 340.0; // Scale factor for temperature
    const float TEMP_OFFSET = 36.53; // Offset for temperature
    const float GRAVITY = 9.80665; // Gravity constant for m/s^2 conversion
    const float ALPHA = 0.98; // Complementary filter constant
    float pitch;
    float roll;
    float yaw;
    float pitchOffset;
    float rollOffset;
    float yawOffset;
};

#endif // IMU_H