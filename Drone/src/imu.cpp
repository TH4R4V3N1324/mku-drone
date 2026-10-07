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
    Wire.write(0x02); // Set DLPF to 2 (gyro 98Hz, ~2.8ms delay; below the 125Hz Nyquist of the 250Hz loop)
    Wire.endTransmission(true);

    Wire.beginTransmission(i2cAddress);
    Wire.write(0x19); // Sample rate divider register
    Wire.write(0x00); // Set sample rate to 1kHz
    Wire.endTransmission(true);

    Wire.beginTransmission(i2cAddress);
    Wire.write(0x1B); // Gyroscope configuration register
    Wire.write(0x10); // Set full scale range to +/- 1000 degrees/sec
    Wire.endTransmission(true);

    Wire.beginTransmission(i2cAddress);
    Wire.write(0x1C); // Accelerometer configuration register
    Wire.write(0x10); // Set full scale range to +/- 8g (headroom so motor vibration doesn't clip)
    Wire.endTransmission(true);

    lastReadTime = micros();
}

/**
 * @brief Calibrate the IMU
 * @details Averages stationary samples to find gyro bias, level accelerometer angles, and Z-axis bias.
 * The drone must be still and on the surface that should read as level.
 * @return None
 */
void IMU::calibrate() {
    const int numSamples = 2000;
    float sumGyroX = 0.0f;
    float sumGyroY = 0.0f;
    float sumGyroZ = 0.0f;
    float sumAccelPitch = 0.0f;
    float sumAccelRoll = 0.0f;
    float sumAccelZ = 0.0f;

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

        float accelPitch, accelRoll;
        accelAngles(ax / ACCEL_SCALE, ay / ACCEL_SCALE, az / ACCEL_SCALE, accelPitch, accelRoll);
        sumAccelPitch += accelPitch;
        sumAccelRoll += accelRoll;
        sumAccelZ += (az / ACCEL_SCALE) * GRAVITY;
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
    accelZOffset = (sumAccelZ / numSamples) - GRAVITY;
    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    lastReadTime = micros();
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
    if (Wire.requestFrom(i2cAddress, 14, true) != 14) {
        return;
    } // Request 14 bytes of data

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    data.accelX = (ax / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.accelY = (ay / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.accelZ = (az / ACCEL_SCALE) * GRAVITY - accelZOffset; // Convert to m/s^2 and remove stationary bias
    data.temperature = (temp / TEMP_SCALE) + TEMP_OFFSET; // Convert to degrees Celsius

    data.gyroX = (gx / GYRO_SCALE) - gyroOffsetX;
    data.gyroY = (gy / GYRO_SCALE) - gyroOffsetY;
    data.gyroZ = (gz / GYRO_SCALE) - gyroOffsetZ;

    // Same raw-g computation as calibrate(), so the offsets cancel exactly
    float accelPitch, accelRoll;
    accelAngles(ax / ACCEL_SCALE, ay / ACCEL_SCALE, az / ACCEL_SCALE, accelPitch, accelRoll);
    calculateOrientation(accelPitch - accelPitchOffset, accelRoll - accelRollOffset);
}

/**
 * @brief Compute tilt angles from a gravity vector
 * @param[in] ax X acceleration (any unit, all three axes the same)
 * @param[in] ay Y acceleration
 * @param[in] az Z acceleration
 * @param[out] pitchDeg Pitch angle in degrees (rotation about X)
 * @param[out] rollDeg Roll angle in degrees (rotation about Y)
 * @return None
 */
void IMU::accelAngles(float ax, float ay, float az, float& pitchDeg, float& rollDeg) {
    pitchDeg = atan2(ay, az) * RAD_TO_DEG;
    rollDeg = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}

/**
 * @brief Calculate the orientation (pitch, roll, yaw) of the IMU
 * @details Complementary filter: integrates the gyro and pulls slowly towards the
 * accelerometer angle with time constant ANGLE_FILTER_TAU. Accelerometer vibration
 * is attenuated by the same time constant, so it needs no separate filter.
 * @param[in] accelPitch Offset-corrected accelerometer pitch in degrees
 * @param[in] accelRoll Offset-corrected accelerometer roll in degrees
 * @return None
 */
void IMU::calculateOrientation(float accelPitch, float accelRoll) {
    unsigned long now = micros();
    float dt = (now - lastReadTime) / 1000000.0f;
    lastReadTime = now;

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    const float alpha = ANGLE_FILTER_TAU / (ANGLE_FILTER_TAU + dt);
    float gyroPitch = pitch + data.gyroX * dt;
    float gyroRoll = roll + data.gyroY * dt;

    pitch = alpha * gyroPitch + (1.0f - alpha) * accelPitch;
    roll = alpha * gyroRoll + (1.0f - alpha) * accelRoll;
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