#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

IMU imu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  imu.init(0x68); // Initialize IMU with I2C address 0x68
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.readData();
  imu.printOrientation();
}
