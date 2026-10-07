#include "calibrationStore.h"
#include <EEPROM.h>

namespace {
constexpr int EEPROM_ADDRESS = 0;
constexpr uint16_t MAGIC = 0x4C56;              // "LV"
constexpr uint8_t VERSION = 1;                  // bump if LevelCalibration changes
constexpr float MAX_LEVEL_OFFSET_DEG = 15.0f;   // anything larger means a bad calibration or mounting
constexpr float MAX_ACCEL_Z_OFFSET = 3.0f;      // m/s^2

struct StoredCalibration {
    uint16_t magic;
    uint8_t version;
    IMU::LevelCalibration level;
    uint8_t checksum;
};

/**
 * @brief Rotate-and-XOR checksum over every byte before the checksum field
 */
uint8_t computeChecksum(const StoredCalibration& stored) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&stored);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(StoredCalibration) - sizeof(stored.checksum); ++i) {
        sum = static_cast<uint8_t>((sum << 1) | (sum >> 7)) ^ bytes[i];
    }
    return sum;
}

bool isPlausible(const IMU::LevelCalibration& level) {
    return isfinite(level.pitchOffset) && isfinite(level.rollOffset) && isfinite(level.accelZOffset) &&
           fabsf(level.pitchOffset) <= MAX_LEVEL_OFFSET_DEG &&
           fabsf(level.rollOffset) <= MAX_LEVEL_OFFSET_DEG &&
           fabsf(level.accelZOffset) <= MAX_ACCEL_Z_OFFSET;
}
}

bool loadLevelCalibration(IMU::LevelCalibration& level) {
    StoredCalibration stored;
    EEPROM.get(EEPROM_ADDRESS, stored);
    if (stored.magic != MAGIC || stored.version != VERSION ||
        stored.checksum != computeChecksum(stored) || !isPlausible(stored.level)) {
        return false;
    }
    level = stored.level;
    return true;
}

void saveLevelCalibration(const IMU::LevelCalibration& level) {
    StoredCalibration stored;
    stored.magic = MAGIC;
    stored.version = VERSION;
    stored.level = level;
    stored.checksum = computeChecksum(stored);
    EEPROM.put(EEPROM_ADDRESS, stored); // put() only rewrites bytes that changed
}
