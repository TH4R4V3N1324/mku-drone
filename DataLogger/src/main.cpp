#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include <SdFat.h>
#include <SPI.h>

#define SD_CS_PIN 10

IMU imu;
SdFat sd;
File32 flightLog;

int flightNum = 0;
unsigned long lastLogTime = 0;
volatile uint8_t loopTicksPending = 0; // Set by the Timer1 ISR at 500 Hz
bool imuFailureReported = false;

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

void stopWithError(const __FlashStringHelper *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

ISR(TIMER1_COMPA_vect) {
  if (loopTicksPending < 255) {
    loopTicksPending++;
  }
}

// Timer1 in CTC mode: 16 MHz / 8 prescaler / (3999 + 1) = 500 Hz
void startLoopTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  TCNT1 = 0;
  OCR1A = 3999;
  TIMSK1 = _BV(OCIE1A);
  interrupts();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  imu.init(0x68); // Initialize IMU with I2C address 0x68
  if (!imu.calibrateGyro() || !imu.resetOrientation()) {
    stopWithError(F("IMU not responding on I2C 0x68"));
  }
  SPI.begin();
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  // Use a conservative SPI speed for reliable writes with typical SD modules.
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(8))) {
    Serial.println(F("SD card initialization failed on CS D10"));
    Serial.println(F("Expected SPI pins: MOSI D11, MISO D12, SCK D13"));
    stopWithError(F("SdFat error: card.begin() failed"));
  }
  Serial.println(F("SD card initialized"));
  Serial.print(F("FAT type: "));
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
    Serial.print(F("Could not create "));
    Serial.println(filename);
    sd.errorPrint(&Serial, F("Could not create flight log"));
    stopWithError(F("Could not create flight log"));
  }
  Serial.print(F("Logging to "));
  Serial.println(filename);

  startLoopTimer();
}

void loop() {
  if (loopTicksPending == 0) {
    return; // Wait for the next Timer1 tick
  }
  // Single-byte access is atomic on AVR; drop any ticks missed while we were busy
  loopTicksPending = 0;

  unsigned long currentTime = micros();

  // Skip the record on an I2C failure or frozen data; the gap shows up in the timestamps
  if (!imu.readData()) {
    if (!imuFailureReported) {
      Serial.println(F("IMU read failed, skipping samples"));
      imuFailureReported = true;
    }
    return;
  }

  BinaryLogRecord record = {
    static_cast<uint32_t>(currentTime),
    imu.getPitch(),
    imu.getRoll(),
    imu.getYaw(),
    imu.getAccelX(),
    imu.getAccelY(),
    imu.getAccelZ(),
    imu.getGyroX(),
    imu.getGyroY(),
    imu.getGyroZ()
  };
  // SdFat buffers this in its 512-byte sector cache and writes each sector as it fills
  if (flightLog.write(&record, sizeof(record)) != sizeof(record)) {
    stopWithError(F("Could not write flight log data"));
  }

  // Flush the log to the SD card
  unsigned long time = millis();
  if (time - lastLogTime >= 5000) {
    flightLog.flush(); // Ensure data is written to the SD card
    lastLogTime = time;
  }
}
