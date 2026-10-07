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
 * @brief Average raw samples while checking that the drone stays still
 * @param[in] numSamples Number of samples to average (about 1.5 ms each)
 * @param[out] result Mean raw gyro rates, accelerometer angles and Z acceleration, plus the gyro spread
 * @return True on success, false if an I2C read failed
 */
bool IMU::sampleStill(uint16_t numSamples, StillSample& result) {
    float sumGyroX = 0.0f, sumGyroY = 0.0f, sumGyroZ = 0.0f;
    float sumAccelPitch = 0.0f, sumAccelRoll = 0.0f, sumAccelZ = 0.0f;
    float minGyro[3] = {1e6f, 1e6f, 1e6f};
    float maxGyro[3] = {-1e6f, -1e6f, -1e6f};

    for (uint16_t i = 0; i < numSamples; ++i) {
        RawSample raw;
        if (!readRaw(raw)) {
            return false;
        }

        const float gyro[3] = {raw.gx / GYRO_SCALE, raw.gy / GYRO_SCALE, raw.gz / GYRO_SCALE};
        for (uint8_t axis = 0; axis < 3; ++axis) {
            minGyro[axis] = fminf(minGyro[axis], gyro[axis]);
            maxGyro[axis] = fmaxf(maxGyro[axis], gyro[axis]);
        }
        sumGyroX += gyro[0];
        sumGyroY += gyro[1];
        sumGyroZ += gyro[2];

        float accelPitch, accelRoll;
        accelAngles(raw.ax / ACCEL_SCALE, raw.ay / ACCEL_SCALE, raw.az / ACCEL_SCALE, accelPitch, accelRoll);
        sumAccelPitch += accelPitch;
        sumAccelRoll += accelRoll;
        sumAccelZ += (raw.az / ACCEL_SCALE) * GRAVITY;
        delay(1);
    }

    result.gyroX = sumGyroX / numSamples;
    result.gyroY = sumGyroY / numSamples;
    result.gyroZ = sumGyroZ / numSamples;
    result.gyroRange = fmaxf(maxGyro[0] - minGyro[0], fmaxf(maxGyro[1] - minGyro[1], maxGyro[2] - minGyro[2]));
    result.accelPitch = sumAccelPitch / numSamples;
    result.accelRoll = sumAccelRoll / numSamples;
    result.accelZ = sumAccelZ / numSamples;
    return true;
}

/**
 * @brief Measure the gyro bias, repeating until the drone is held still
 * @details Run at every boot: gyro bias drifts with temperature. Tilt doesn't matter,
 * so this works on uneven ground.
 * @return True on success, false if an I2C read failed
 */
bool IMU::calibrateGyro() {
    StillSample sample;
    while (true) {
        if (!sampleStill(GYRO_CAL_SAMPLES, sample)) {
            return false;
        }
        if (sample.gyroRange <= STILL_GYRO_RANGE_DPS) {
            break;
        }
        Serial.println(F("Movement detected during gyro calibration, retrying - keep the drone still."));
    }

    gyroOffsetX = sample.gyroX;
    gyroOffsetY = sample.gyroY;
    gyroOffsetZ = sample.gyroZ;
    return true;
}

/**
 * @brief Measure the accelerometer offsets that define level
 * @details The frame must be level and still. Does not apply the result; see setLevelCalibration().
 * @param[out] level The measured level offsets
 * @param[in] numSamples Number of samples to average (about 1.5 ms each)
 * @param[out] moved True if the drone moved during the measurement (result should be discarded)
 * @return True on success, false if an I2C read failed
 */
bool IMU::measureLevel(LevelCalibration& level, uint16_t numSamples, bool& moved) {
    StillSample sample;
    if (!sampleStill(numSamples, sample)) {
        return false;
    }
    moved = sample.gyroRange > STILL_GYRO_RANGE_DPS;
    level.pitchOffset = sample.accelPitch;
    level.rollOffset = sample.accelRoll;
    level.accelZOffset = sample.accelZ - GRAVITY;
    return true;
}

/**
 * @brief Apply level offsets (from EEPROM or measureLevel())
 * @param[in] level The level offsets to use
 * @return None
 */
void IMU::setLevelCalibration(const LevelCalibration& level) {
    accelPitchOffset = level.pitchOffset;
    accelRollOffset = level.rollOffset;
    accelZOffset = level.accelZOffset;
}

/**
 * @brief Start the angle estimate from the current accelerometer reading
 * @details Call after calibration so the complementary filter doesn't start at 0
 * when the drone is sitting on a slope. Yaw is reset to 0.
 * @return True on success, false if an I2C read failed
 */
bool IMU::resetOrientation() {
    RawSample raw;
    if (!readRaw(raw)) {
        return false;
    }
    float accelPitch, accelRoll;
    accelAngles(raw.ax / ACCEL_SCALE, raw.ay / ACCEL_SCALE, raw.az / ACCEL_SCALE, accelPitch, accelRoll);
    pitch = accelPitch - accelPitchOffset;
    roll = accelRoll - accelRollOffset;
    yaw = 0.0f;
    lastReadTime = micros();
    return true;
}

/**
 * @brief Read one raw accelerometer/temperature/gyro sample (registers 0x3B-0x48)
 * @param[out] raw The raw sensor counts
 * @return True on success, false on I2C failure
 */
bool IMU::readRaw(RawSample& raw) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x3B); // Starting register for accelerometer data
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(i2cAddress, 14, true) != 14) {
        return false;
    }

    // Read high byte then low byte in separate statements: the order of two
    // Wire.read() calls inside one expression is unspecified in C++
    auto readWord = []() -> int16_t {
        const uint8_t high = Wire.read();
        const uint8_t low = Wire.read();
        return static_cast<int16_t>((high << 8) | low);
    };
    raw.ax = readWord();
    raw.ay = readWord();
    raw.az = readWord();
    raw.temp = readWord();
    raw.gx = readWord();
    raw.gy = readWord();
    raw.gz = readWord();
    return true;
}

/**
 * @brief Read data from the IMU
 * @details Reads accelerometer, gyroscope, and temperature data from the IMU and updates the internal state.
 * On an I2C failure the previous data and orientation are left unchanged.
 * @return True if a fresh sample was read, false on I2C failure
 */
bool IMU::readData() {
    RawSample raw;
    if (!readRaw(raw)) {
        return false;
    }

    data.accelX = (raw.ax / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.accelY = (raw.ay / ACCEL_SCALE) * GRAVITY; // Convert to m/s^2
    data.accelZ = (raw.az / ACCEL_SCALE) * GRAVITY - accelZOffset; // Convert to m/s^2 and remove stationary bias
    data.temperature = (raw.temp / TEMP_SCALE) + TEMP_OFFSET; // Convert to degrees Celsius

    data.gyroX = (raw.gx / GYRO_SCALE) - gyroOffsetX;
    data.gyroY = (raw.gy / GYRO_SCALE) - gyroOffsetY;
    data.gyroZ = (raw.gz / GYRO_SCALE) - gyroOffsetZ;

    // Same raw-g computation as calibrate(), so the offsets cancel exactly
    float accelPitch, accelRoll;
    accelAngles(raw.ax / ACCEL_SCALE, raw.ay / ACCEL_SCALE, raw.az / ACCEL_SCALE, accelPitch, accelRoll);
    calculateOrientation(accelPitch - accelPitchOffset, accelRoll - accelRollOffset);
    return true;
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