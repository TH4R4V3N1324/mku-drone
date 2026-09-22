#include "imu.h"

void IMU::init() {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    lastReadTime = millis();
}

void IMU::calibrate() {
    // Calibration logic for the IMU
    // This can include reading multiple samples and averaging them to find offsets
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

void IMU::readData() {
    // Implementation for reading IMU data
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(0x3B); // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDRESS, 14, true); // Request 14 bytes of data

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

void IMU::printData() {
    Serial.print("Accel X: "); Serial.print(data.accelX);
    Serial.print(" | Accel Y: "); Serial.print(data.accelY);
    Serial.print(" | Accel Z: "); Serial.print(data.accelZ);
    Serial.print(" | Gyro X: "); Serial.print(data.gyroX);
    Serial.print(" | Gyro Y: "); Serial.print(data.gyroY);
    Serial.print(" | Gyro Z: "); Serial.print(data.gyroZ);
    Serial.print(" | Temperature: "); Serial.println(data.temperature);
}

void IMU::printOrientation() {
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" | Roll: "); Serial.println(roll);
}