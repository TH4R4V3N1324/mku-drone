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

    Wire.beginTransmission(i2cAddress);
    Wire.write(0x1A); // Configuration register
    Wire.write(0x03); // Set DLPF to 3 (44Hz)
    Wire.endTransmission(true);

    Wire.beginTransmission(i2cAddress);
    Wire.write(0x19); // Sample rate divider register
    Wire.write(0x09); // Set sample rate to 100Hz
    Wire.endTransmission(true);

    lastReadTime = millis();
}

/**
 * @brief Calibrate the IMU
 * @details Calibrates the gyroscope by averaging stationary angular-rate samples
 * @return None
 */
void IMU::calibrate() {
    const int numSamples = 2000;
    float sumGyroX = 0.0f;
    float sumGyroY = 0.0f;
    float sumGyroZ = 0.0f;
    float sumAccelPitch = 0.0f;
    float sumAccelRoll = 0.0f;

    for (int i = 0; i < numSamples; ++i) {
        Wire.beginTransmission(i2cAddress);
        Wire.write(0x3B);
        if (Wire.endTransmission(false) != 0 || Wire.requestFrom(i2cAddress, 14, true) != 14) {
            return;
        }

        int16_t ax = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
        int16_t ay = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
        int16_t az = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
        Wire.read();
        Wire.read();
        int16_t gx = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
        int16_t gy = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
        int16_t gz = static_cast<int16_t>(Wire.read() << 8 | Wire.read());

        float accelX = ax / ACCEL_SCALE;
        float accelY = ay / ACCEL_SCALE;
        float accelZ = az / ACCEL_SCALE;
        sumAccelPitch += atan2(accelY, accelZ) * 180.0f / PI;
        sumAccelRoll += atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;
        sumGyroX += gx / GYRO_SCALE;
        sumGyroY += gy / GYRO_SCALE;
        sumGyroZ += gz / GYRO_SCALE;
        delay(5);
    }

    gyroOffsetX = sumGyroX / numSamples;
    gyroOffsetY = sumGyroY / numSamples;
    gyroOffsetZ = sumGyroZ / numSamples;
    accelPitchOffset = sumAccelPitch / numSamples;
    accelRollOffset = sumAccelRoll / numSamples;
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    lastReadTime = micros();
    lastFilterTime = 0;
    filterInitialized = false;
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
    data.temperature = (temp / TEMP_SCALE) + TEMP_OFFSET; // Convert to degrees Celsius
    
    float rawGyroX = (gx / GYRO_SCALE) - gyroOffsetX;
    float rawGyroY = (gy / GYRO_SCALE) - gyroOffsetY;
    float rawGyroZ = (gz / GYRO_SCALE) - gyroOffsetZ;

    unsigned long now = micros();
    float filterDt = lastFilterTime == 0
        ? 0.0f
        : (now - lastFilterTime) / 1000000.0f;
    lastFilterTime = now;

    if (!filterInitialized || filterDt <= 0.0f) {
        filteredGyroX = rawGyroX;
        filteredGyroY = rawGyroY;
        filteredGyroZ = rawGyroZ;
        filterInitialized = true;
    } else {
        const float timeConstant = 1.0f / (2.0f * PI * GYRO_FILTER_CUTOFF_HZ);
        const float filterAlpha = filterDt / (timeConstant + filterDt);
        filteredGyroX += filterAlpha * (rawGyroX - filteredGyroX);
        filteredGyroY += filterAlpha * (rawGyroY - filteredGyroY);
        filteredGyroZ += filterAlpha * (rawGyroZ - filteredGyroZ);
    }

    data.gyroX = filteredGyroX;
    data.gyroY = filteredGyroY;
    data.gyroZ = filteredGyroZ;

    calculateOrientation();
}

/**
 * @brief Calculate the orientation (pitch, roll, yaw) of the IMU
 * @details Uses a complementary filter to combine accelerometer and gyroscope data to calculate pitch, roll, and yaw
 * @return None
 */
void IMU::calculateOrientation() {
    // Calculate pitch and roll using complementary filter
    unsigned long now = micros();
    float dt = (now - lastReadTime) / 1000000.0f;
    lastReadTime = now;

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    float accelPitch = atan2(data.accelY, data.accelZ) * 180.0f / PI - accelPitchOffset;
    float accelRoll = atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * 180.0f / PI - accelRollOffset;
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
    Serial.print(" | Yaw: "); Serial.print(yaw);
    Serial.print(" | Roll: "); Serial.println(roll);
}