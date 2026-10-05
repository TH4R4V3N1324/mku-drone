# DataLogger

PlatformIO firmware for an Arduino Nano that reads an MPU-6050 and records IMU data to a microSD card as binary records.



## Hardware and wiring

- Arduino Nano with the ATmega328P
- MPU-6050 at I2C address `0x68`
- microSD card module compatible with the SdFat library

### MPU-6050

Connect the sensor using the Nano's standard I2C pins:

| MPU-6050 signal | Arduino Nano pin |
| --- | --- |
| SDA | A4 |
| SCL | A5 |
| VCC | 5V |
| GND | GND |

The firmware uses the I2C address `0x68`, so the sensor's address-select pin must be configured accordingly.

### SD card module

The chip-select pin is fixed to D10. The SPI connections are:

| SD signal | Arduino Nano pin |
| --- | --- |
| CS | D10 |
| MOSI | D11 |
| MISO | D12 |
| SCK | D13 |
| VCC | 5V |
| GND | GND |


## Firmware behavior

At startup, the firmware:

1. Starts serial output at `115200` baud.
2. Starts I2C at 400 kHz and initializes the MPU-6050 for its 1 kHz internal sample rate with the digital low-pass filter disabled.
3. Calibrates the sensor using 2,000 stationary samples. Keep the board still during this step.
4. Initializes the SD card on chip select D10 at a conservative 8 MHz SPI speed for reliable writes.
5. Reads `logNum.txt`, increments its value, and writes the updated value back.
6. Creates a new binary file named `FlightN.bin`, where `N` is the incremented flight number.

If SD initialization or file creation fails, the firmware prints an error and stops in an infinite loop. Existing log files are not overwritten because the flight number is persisted in `logNum.txt`.

The main loop reads the IMU, calculates orientation, and appends one fixed 40-byte binary record to a 128-byte RAM buffer. Each record contains a `uint32_t` microsecond timestamp followed by nine IEEE-754 `float32` values: pitch, roll, yaw, acceleration X/Y/Z, and gyro X/Y/Z. The buffer is written to the card as raw bytes when full and is flushed to the filesystem every five seconds. This avoids CSV conversion and substantially reduces the data size. Serial timing output is disabled in the hot loop because transmitting diagnostics at 115200 baud significantly reduces the maximum logging rate. Remove the SD card only after stopping or powering down the logger so the last buffered data is not lost.

## Binary format

Binary files contain consecutive 40-byte little-endian records with no header:

```text
uint32 timestamp_us
float32 pitch
float32 roll
float32 yaw
float32 accel_x
float32 accel_y
float32 accel_z
float32 gyro_x
float32 gyro_y
float32 gyro_z
```

For example, Python can decode a file with:

```python
import numpy as np

record_dtype = np.dtype([
    ("timestamp_us", "<u4"),
    ("pitch", "<f4"),
    ("roll", "<f4"),
    ("yaw", "<f4"),
    ("accel_x", "<f4"),
    ("accel_y", "<f4"),
    ("accel_z", "<f4"),
    ("gyro_x", "<f4"),
    ("gyro_y", "<f4"),
    ("gyro_z", "<f4"),
])
records = np.fromfile("FlightN.bin", dtype=record_dtype)
```

| Column | Unit | Description |
| --- | --- | --- |
| Timestamp | us | Arduino `micros()` value since boot |
| Pitch | degrees | Complementary-filter pitch estimate |
| Roll | degrees | Complementary-filter roll estimate |
| Yaw | degrees | Gyroscope-integrated yaw estimate; it will drift |
| Accel X/Y/Z | m/s^2 | Accelerometer readings |
| Gyro X/Y/Z | degrees/s | Bias-calibrated and low-pass-filtered angular rates |

The MPU-6050 temperature is read by the IMU class but is not currently written to the binary record.

## Build, upload, and monitor

Install [PlatformIO](https://platformio.org/) through the CLI or the VS Code extension. From this directory, run:

```text
pio run
pio run --target upload
pio device monitor --baud 115200
```

The project is configured for the `nanoatmega328` board and the Arduino framework. The `SdFat` and `SPI` dependencies are declared in `platformio.ini`.

On a successful startup, the serial monitor reports the SD-card status and the name of the active log file. Remove the SD card only after stopping or powering down the logger so the last buffered data is not lost.

## Project layout

```text
include/imu.h       MPU-6050 interface and data accessors
src/imu.cpp         Sensor initialization, calibration, filtering, and orientation
src/main.cpp        SdFat setup, file naming, and buffered binary logging loop
platformio.ini      Nano, framework, monitor, and library configuration
```