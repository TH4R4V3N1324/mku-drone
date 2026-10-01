#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include <SD.h>
#include <SPI.h>

IMU imu;
File flightLog;

const int chipSelect = 10;
int flightNum = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  imu.init(0x68); // Initialize IMU with I2C address 0x68
  imu.calibrate(); // Calibrate the IMU
  SD.begin();
  
  File logNumFile;
  logNumFile = SD.open("logNum.txt", FILE_READ);
  if (logNumFile) {
    flightNum = logNumFile.parseInt()+1;
    logNumFile.write(String(flightNum).c_str());
    logNumFile.close();
  };
  

  String filename = "flightNumber_" + String(flightNum) + ".csv";
  flightLog = SD.open(filename.c_str(), FILE_WRITE);

  //Creates headers for CSV file
  flightLog.print("Current Time ,");
  flightLog.print("Pitch ,");
  flightLog.print("Roll ,"); 
  flightLog.print("Yaw ,");
  flightLog.print("Accel X ,");
  flightLog.print("Accel Y ,");
  flightLog.print("Accel Z ,");
  flightLog.print("Gyro X ,");
  flightLog.print("Gyro Y ,");
  flightLog.print("Gyro Z ,");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.readData();
  
  unsigned long currentTime = millis();

  float pitch = imu.getPitch();
  float roll = imu.getRoll();
  float yaw = imu.getYaw();
  float accelX = imu.getAccelX();
  float accelY = imu.getAccelY();
  float accelZ = imu.getAccelZ();
  float gyroX = imu.getGyroX();
  float gyroY = imu.getGyroY();
  float gyroZ = imu.getGyroZ();

  // Log data to SD card
  flightLog.print(currentTime);
  flightLog.print(",");
  flightLog.print(pitch);
  flightLog.print(",");
  flightLog.print(roll);
  flightLog.print(",");
  flightLog.print(yaw);
  flightLog.print(",");
  flightLog.print(accelX);
  flightLog.print(",");
  flightLog.print(accelY);
  flightLog.print(",");
  flightLog.print(accelZ);
  flightLog.print(",");
  flightLog.print(gyroX);
  flightLog.print(",");
  flightLog.print(gyroY);
  flightLog.print(",");
  flightLog.println(gyroZ);

}
