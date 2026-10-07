#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <Arduino.h>
#include <SPI.h>

/**
 * @brief Driver for the PixArt PMW3901 optical flow sensor (SPI)
 */
class OpticalFlow {
public:
    OpticalFlow() :
        csPin(10),
        spiSettings(2000000, MSBFIRST, SPI_MODE3),
        deltaX(0),
        deltaY(0),
        quality(0),
        shutter(0) {}
    bool init(uint8_t csPin);
    bool readData();
    void printData();
    /**
     * @brief Get the X motion since the last read.
     * @return Raw flow counts (not yet scaled by height or compensated for rotation).
     */
    int16_t getDeltaX() const { return deltaX; }
    /**
     * @brief Get the Y motion since the last read.
     * @return Raw flow counts (not yet scaled by height or compensated for rotation).
     */
    int16_t getDeltaY() const { return deltaY; }
    /**
     * @brief Get the surface quality (number of features the sensor is tracking).
     * @return 0-255, low values mean the deltas are unreliable.
     */
    uint8_t getQuality() const { return quality; }

private:
    struct MotionBurst {
        uint8_t motion;
        uint8_t observation;
        uint8_t deltaXLow, deltaXHigh;
        uint8_t deltaYLow, deltaYHigh;
        uint8_t squal;
        uint8_t rawDataSum;
        uint8_t maxRawData;
        uint8_t minRawData;
        uint8_t shutterUpper, shutterLower;
    };
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readMotionBurst(MotionBurst& burst);
    void writePerformanceSettings();
    uint8_t csPin;
    SPISettings spiSettings;
    int16_t deltaX;
    int16_t deltaY;
    uint8_t quality;
    uint16_t shutter;
    static constexpr uint8_t REG_PRODUCT_ID = 0x00;
    static constexpr uint8_t REG_MOTION = 0x02;
    static constexpr uint8_t REG_DELTA_X_L = 0x03;
    static constexpr uint8_t REG_INVERSE_PRODUCT_ID = 0x5F;
    static constexpr uint8_t REG_POWER_UP_RESET = 0x3A;
    static constexpr uint8_t REG_MOTION_BURST = 0x16;
    static constexpr uint8_t PRODUCT_ID = 0x49;
    static constexpr uint8_t INVERSE_PRODUCT_ID = 0xB6;
    static constexpr uint8_t MOTION_BIT = 0x80;          // set in Motion when new deltas are ready
    static constexpr uint8_t MIN_QUALITY = 0x19;          // below this, or with shutter maxed, treat deltas as noise
    static constexpr uint8_t MAX_SHUTTER_UPPER = 0x1F;
};

#endif // OPTICAL_FLOW_H
