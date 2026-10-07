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
    /**
     * @brief Accelerometer offsets that define "level" (a property of how the IMU is mounted)
     */
    struct LevelCalibration {
        float pitchOffset;   // deg, accelerometer pitch when the frame is level
        float rollOffset;    // deg, accelerometer roll when the frame is level
        float accelZOffset;  // m/s^2, Z reading minus gravity when level
    };

    IMU() :
        data{},
        lastReadTime(0),
        i2cAddress(IMU_ADDRESS),
        pitch(0.0f),
        roll(0.0f),
        yaw(0.0f),
        gyroOffsetX(0.0f),
        gyroOffsetY(0.0f),
        gyroOffsetZ(0.0f),
        accelPitchOffset(0.0f),
        accelRollOffset(0.0f),
        accelZOffset(0.0f) {}
    void init(int address = IMU_ADDRESS);
    bool calibrateGyro();
    bool measureLevel(LevelCalibration& level, uint16_t numSamples, bool& moved);
    void setLevelCalibration(const LevelCalibration& level);
    LevelCalibration getLevelCalibration() const { return {accelPitchOffset, accelRollOffset, accelZOffset}; }
    bool resetOrientation();
    bool readData();
    void printData();
    void printOrientation();
    /**
    * @brief Get the X-axis acceleration.
    * @return Acceleration in m/s^2.
     */
    float getAccelX() const { return data.accelX; }
    /**
    * @brief Get the Y-axis acceleration.
    * @return Acceleration in m/s^2.
     */
    float getAccelY() const { return data.accelY; }
    /**
    * @brief Get the Z-axis acceleration.
    * @return Acceleration in m/s^2.
     */
    float getAccelZ() const { return data.accelZ; }
    /**
    * @brief Get the X-axis angular velocity.
    * @return Angular velocity in degrees/s.
     */
    float getGyroX() const { return data.gyroX; }
    /**
    * @brief Get the Y-axis angular velocity.
    * @return Angular velocity in degrees/s.
     */
    float getGyroY() const { return data.gyroY; }
    /**
    * @brief Get the Z-axis angular velocity.
    * @return Angular velocity in degrees/s.
     */
    float getGyroZ() const { return data.gyroZ; }
    /**
    * @brief Get the sensor temperature.
    * @return Temperature in degrees Celsius.
     */
    float getTemperature() const { return data.temperature; }
    /**
    * @brief Get the calculated pitch angle.
    * @return Pitch in degrees.
     */
    float getPitch() const { return pitch; }
    /**
    * @brief Get the calculated roll angle.
    * @return Roll in degrees.
     */
    float getRoll() const { return roll; }
    /**
    * @brief Get the integrated yaw angle.
    * @return Yaw in degrees. This value can drift over time.
     */
    float getYaw() const { return yaw; }
private:
    struct RawSample {
        int16_t ax, ay, az, temp, gx, gy, gz;
    };
    struct StillSample {
        float gyroX, gyroY, gyroZ;   // deg/s, mean raw rate
        float gyroRange;             // deg/s, largest max-min spread on any gyro axis (motion check)
        float accelPitch, accelRoll; // deg, mean raw accelerometer angles
        float accelZ;                // m/s^2, mean raw Z acceleration
    };
    bool readRaw(RawSample& raw);
    bool sampleStill(uint16_t numSamples, StillSample& result);
    void calculateOrientation(float accelPitch, float accelRoll);
    static void accelAngles(float ax, float ay, float az, float& pitchDeg, float& rollDeg);
    IMUData data;
    unsigned long lastReadTime;
    int i2cAddress;
    static constexpr float ACCEL_SCALE = 4096.0f; // Scale factor for accelerometer (+/- 8g)
    static constexpr float GYRO_SCALE = 32.8f; // Scale factor for gyroscope (+/- 1000 deg/s)
    static constexpr float TEMP_SCALE = 340.0f; // Scale factor for temperature
    static constexpr float TEMP_OFFSET = 36.53f; // Offset for temperature
    static constexpr float GRAVITY = 9.80665f; // Gravity constant for m/s^2 conversion
    static constexpr float ANGLE_FILTER_TAU = 1.0f; // s, complementary filter time constant (higher = trust gyro longer)
    static constexpr float STILL_GYRO_RANGE_DPS = 2.0f; // gyro spread above this during sampling means the drone moved
    static constexpr uint16_t GYRO_CAL_SAMPLES = 1000;
    float pitch;
    float roll;
    float yaw;
    float gyroOffsetX;
    float gyroOffsetY;
    float gyroOffsetZ;
    float accelPitchOffset;
    float accelRollOffset;
    float accelZOffset;
};

#endif // IMU_H