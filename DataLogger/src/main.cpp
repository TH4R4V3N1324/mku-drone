#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "log_buffer.h"
#include <SdFat.h>
#include <SPI.h>

#define SD_CS_PIN 10

IMU imu;
SdFat sd;
File32 flightLog;

int flightNum = 0;
unsigned long lastLogTime = 0;

struct BinaryLogRecord {
  uint32_t timestampUs;
  float pitch;
  float roll;
  float yaw;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

static_assert(sizeof(BinaryLogRecord) == 40, "Unexpected binary log record size");

LogBuffer logBuffer;

void stopWithError(const __FlashStringHelper *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  imu.init(0x68); // Initialize IMU with I2C address 0x68
  imu.calibrate(); // Calibrate the IMU
  SPI.begin();
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  // Use a conservative SPI speed for reliable writes with typical SD modules.
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(8))) {
    Serial.println("SD card initialization failed on CS D10");
    Serial.println("Expected SPI pins: MOSI D11, MISO D12, SCK D13");
    stopWithError(F("SdFat error: card.begin() failed"));
  }
  Serial.println("SD card initialized");
  Serial.print("FAT type: ");
  Serial.println(sd.fatType());
  
  File32 logNumFile;
  logNumFile = sd.open("logNum.txt", O_RDONLY);
  if (logNumFile) {
    flightNum = logNumFile.parseInt()+1;
    logNumFile.close();
  }

  logNumFile = sd.open("logNum.txt", O_RDWR | O_CREAT | O_TRUNC);
  if (!logNumFile) {
    sd.errorPrint(&Serial, F("Could not create logNum.txt"));
    stopWithError(F("Could not create logNum.txt"));
  }
  logNumFile.println(flightNum);
  logNumFile.close();
  

  char filename[24];
  strcpy_P(filename, PSTR("Flight"));
  itoa(flightNum, filename + 6, 10);
  strcat_P(filename, PSTR(".bin"));
  flightLog = sd.open(filename, O_RDWR | O_CREAT | O_AT_END);
  if (!flightLog) {
    Serial.print("Could not create ");
    Serial.println(filename);
    sd.errorPrint(&Serial, F("Could not create flight log"));
    stopWithError(F("Could not create flight log"));
  }
  logBuffer.attach(flightLog);
  Serial.print("Logging to ");
  Serial.println(filename);
}

void loop() {
  imu.readData();
  
  unsigned long currentTime = micros();

  float pitch = imu.getPitch();
  float roll = imu.getRoll();
  float yaw = imu.getYaw();
  float accelX = imu.getAccelX();
  float accelY = imu.getAccelY();
  float accelZ = imu.getAccelZ();
  float gyroX = imu.getGyroX();
  float gyroY = imu.getGyroY();
  float gyroZ = imu.getGyroZ();

  BinaryLogRecord record = {
    static_cast<uint32_t>(currentTime),
    pitch,
    roll,
    yaw,
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ
  };
  if (logBuffer.write(
        reinterpret_cast<const uint8_t *>(&record), sizeof(record)) != sizeof(record)) {
    stopWithError(F("Could not buffer flight log data"));
  }

  // Flush the log to the SD card
  unsigned long time = millis();
  if (time - lastLogTime >= 5000) {
    if (!logBuffer.drain(flightLog)) {
      stopWithError(F("Could not write flight log data"));
    }
    flightLog.flush(); // Ensure data is written to the SD card
    lastLogTime = time;
  }
}
