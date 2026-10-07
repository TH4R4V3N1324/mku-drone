#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

#include "imu.h"

/**
 * @brief Load the IMU level calibration saved in EEPROM
 * @param[out] level The stored offsets (only written on success)
 * @return True if a valid calibration was found (correct version, checksum and plausible values)
 */
bool loadLevelCalibration(IMU::LevelCalibration& level);

/**
 * @brief Save the IMU level calibration to EEPROM
 * @param[in] level The offsets to store
 * @return None
 */
void saveLevelCalibration(const IMU::LevelCalibration& level);

#endif // CALIBRATION_STORE_H
