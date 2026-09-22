#include "imu.h"

/**
 * @brief Initialize the IMU
 * @details Initializes the IMU by setting up the I2C communication and waking up the MPU-6050
 * @param[in] address The I2C address of the IMU
 * @return None
 */
void IMU::init(int address) {
    this->i2cAddress = address;
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    lastReadTime = millis();
}

/**
 * @brief Calibrate the IMU
 * @details Calibrates the IMU by reading multiple samples and averaging them to find offsets for pitch, roll, and yaw
 * @return None
 */
void IMU::calibrate() {
    this->pitchOffset = 0.0f;
    this->rollOffset = 0.0f;
    this->yawOffset = 0.0f;

    // Read a number of samples and average them to find the offsets
    for (int i = 0; i < 100; i++) {
        readData();
        this->pitchOffset += pitch;
        this->rollOffset += roll;
        this->yawOffset += yaw;
        delay(10);
    }
    this->pitchOffset /= 100.0f;
    this->rollOffset /= 100.0f;
    this->yawOffset /= 100.0f;
}

/**
 * @brief Read data from the IMU
 * @details Reads accelerometer, gyroscope, and temperature data from the IMU and updates the internal state
 * @return None
 */
void IMU::readData() {
    // Implementation for reading IMU data
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x3B); // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(i2cAddress, 14, true); // Request 14 bytes of data

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    data.accelX = (ax / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.accelY = (ay / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.accelZ = (az / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.gyroX = (gx / GYRO_SCALE); // Convert to degrees/s
    data.gyroY = (gy / GYRO_SCALE); // Convert to degrees/s
    data.gyroZ = (gz / GYRO_SCALE); // Convert to degrees
    data.temperature = (temp / TEMP_SCALE) + TEMP_OFFSET; // Convert to degrees Celsius

    calculateOrientation();
}

/**
 * @brief Calculate the orientation (pitch, roll, yaw) of the IMU
 * @details Uses a complementary filter to combine accelerometer and gyroscope data to calculate pitch, roll, and yaw
 * @return None
 */
void IMU::calculateOrientation() {
    // Calculate pitch and roll using complementary filter
    unsigned long now = millis();
    float dt = (now - lastReadTime) / 1000.0f;
    lastReadTime = now;

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    float accelPitch = atan2(data.accelY, data.accelZ) * 180 / PI;
    float accelRoll = atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * 180 / PI;
    float gyroPitch = pitch + data.gyroX * dt;
    float gyroRoll = roll + data.gyroY * dt;

    pitch = ALPHA * gyroPitch + (1 - ALPHA) * accelPitch;
    roll = ALPHA * gyroRoll + (1 - ALPHA) * accelRoll;
    yaw += data.gyroZ * dt; // Integrate yaw from gyroscope
}
/**
 * @brief Print the raw data from the IMU
 * @details Prints the accelerometer, gyroscope, and temperature data to the serial monitor
 * @return None
 */
void IMU::printData() {
    Serial.print("Accel X: "); Serial.print(data.accelX);
    Serial.print(" | Accel Y: "); Serial.print(data.accelY);
    Serial.print(" | Accel Z: "); Serial.print(data.accelZ);
    Serial.print(" | Gyro X: "); Serial.print(data.gyroX);
    Serial.print(" | Gyro Y: "); Serial.print(data.gyroY);
    Serial.print(" | Gyro Z: "); Serial.print(data.gyroZ);
    Serial.print(" | Temperature: "); Serial.println(data.temperature);
}

/**
 * @brief Print the orientation data from the IMU
 * @details Prints the pitch, roll, and yaw data to the serial monitor
 * @return None
 */
void IMU::printOrientation() {
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" | Roll: "); Serial.println(roll);
}