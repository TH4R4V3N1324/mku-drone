#include "opticalFlow.h"

/**
 * @brief Initialize the optical flow sensor
 * @details Resets the PMW3901, checks its product ID and loads PixArt's performance settings.
 *          SPI.begin() must be called first.
 * @param[in] csPin Chip select pin
 * @return True if the sensor answered with the right product ID, false otherwise
 */
bool OpticalFlow::init(uint8_t csPin) {
    this->csPin = csPin;
    pinMode(csPin, OUTPUT);

    // Pulse CS to reset the SPI port
    digitalWrite(csPin, HIGH);
    delay(1);
    digitalWrite(csPin, LOW);
    delay(1);
    digitalWrite(csPin, HIGH);
    delay(1);

    writeRegister(REG_POWER_UP_RESET, 0x5A);
    delay(5);

    if (readRegister(REG_PRODUCT_ID) != PRODUCT_ID ||
        readRegister(REG_INVERSE_PRODUCT_ID) != INVERSE_PRODUCT_ID) {
        return false;
    }

    // Read the motion registers once to clear any stale data (datasheet power-up sequence)
    readRegister(REG_MOTION);
    for (uint8_t reg = REG_DELTA_X_L; reg < REG_DELTA_X_L + 4; ++reg) {
        readRegister(reg);
    }
    delay(1);

    writePerformanceSettings();
    return true;
}

/**
 * @brief Read the motion since the last call
 * @details Deltas are zeroed when the sensor reports no motion or the surface is too poor to track
 * @return True if the frame is valid, false if the surface quality is too low to trust
 */
bool OpticalFlow::readData() {
    MotionBurst burst;
    readMotionBurst(burst);

    quality = burst.squal;
    shutter = ((uint16_t)burst.shutterUpper << 8) | burst.shutterLower;

    if (quality < MIN_QUALITY && burst.shutterUpper == MAX_SHUTTER_UPPER) {
        deltaX = 0;
        deltaY = 0;
        return false;
    }

    if (burst.motion & MOTION_BIT) {
        deltaX = (int16_t)(((uint16_t)burst.deltaXHigh << 8) | burst.deltaXLow);
        deltaY = (int16_t)(((uint16_t)burst.deltaYHigh << 8) | burst.deltaYLow);
    } else {
        deltaX = 0;
        deltaY = 0;
    }
    return true;
}

void OpticalFlow::printData() {
    Serial.print(F("dX: "));
    Serial.print(deltaX);
    Serial.print(F("\tdY: "));
    Serial.print(deltaY);
    Serial.print(F("\tSQUAL: "));
    Serial.print(quality);
    Serial.print(F("\tShutter: "));
    Serial.println(shutter);
}

/**
 * @brief Read one register
 * @param[in] reg Register address
 * @return Register value
 */
uint8_t OpticalFlow::readRegister(uint8_t reg) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(csPin, LOW);

    SPI.transfer(reg & 0x7F); // MSB clear = read
    delayMicroseconds(50);    // tSRAD: sensor needs time before the data byte
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(200);   // tSRW/tSRR: gap before the next command
    return value;
}

/**
 * @brief Write one register
 * @param[in] reg Register address
 * @param[in] value Value to write
 */
void OpticalFlow::writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(csPin, LOW);

    SPI.transfer(reg | 0x80); // MSB set = write
    SPI.transfer(value);

    delayMicroseconds(50);    // tSCLK-NCS
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(200);   // tSWW/tSWR: gap before the next command
}

/**
 * @brief Read Motion, Delta_X/Y, SQUAL and Shutter in one transaction
 * @param[out] burst The 12 bytes the sensor returns, in register order
 */
void OpticalFlow::readMotionBurst(MotionBurst& burst) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(csPin, LOW);

    SPI.transfer(REG_MOTION_BURST);
    delayMicroseconds(50);    // tSRAD_MOTBR
    uint8_t* bytes = (uint8_t*)&burst;
    for (uint8_t i = 0; i < sizeof(MotionBurst); ++i) {
        bytes[i] = SPI.transfer(0x00);
    }

    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(200);
}

/**
 * @brief Load PixArt's undocumented tuning registers
 * @details Copied from the PMW3901 datasheet / Bitcraze driver. Register 0x7F selects a bank,
 *          so the sequence must finish back on bank 0 or later reads hit the wrong registers.
 */
void OpticalFlow::writePerformanceSettings() {
    static const uint8_t settings[][2] PROGMEM = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
        {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
        {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
        {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
        {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
        {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
        {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
        {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11},
        {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78},
        {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
        {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x60}, {0x66, 0x08},
        {0x63, 0x78}, {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07},
        {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F},
        {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
        {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00},
    };
    static const uint8_t settingsAfterDelay[][2] PROGMEM = {
        {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06},
        {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0},
        {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8},
        {0x5A, 0x50}, {0x40, 0x80},
    };

    for (uint8_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        writeRegister(pgm_read_byte(&settings[i][0]), pgm_read_byte(&settings[i][1]));
    }
    delay(100);
    for (uint8_t i = 0; i < sizeof(settingsAfterDelay) / sizeof(settingsAfterDelay[0]); ++i) {
        writeRegister(pgm_read_byte(&settingsAfterDelay[i][0]), pgm_read_byte(&settingsAfterDelay[i][1]));
    }
}
