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
2. Starts I2C at 400 kHz and initializes the MPU-6050 with a 1 kHz internal sample rate, the digital low-pass filter at 98 Hz (gyro), and ranges of ±8 g and ±1000 deg/s.
3. Measures the gyro bias from 1,000 stationary samples, repeating the measurement if the board moves, then seeds pitch and roll from the accelerometer. Keep the board still during this step. If the IMU does not respond, the firmware prints an error and stops.
4. Initializes the SD card on chip select D10 at a conservative 8 MHz SPI speed for reliable writes.
5. Reads `logNum.txt`, increments its value, and writes the updated value back.
6. Creates a new binary file named `FlightN.bin`, where `N` is the incremented flight number.

If SD initialization or file creation fails, the firmware prints an error and stops in an infinite loop. Existing log files are not overwritten because the flight number is persisted in `logNum.txt`.

The main loop runs at a fixed `500 Hz`, paced by a Timer1 interrupt. Ticks missed while a loop is busy (for example, during an SD write) are dropped rather than run back to back. Each loop reads the IMU, updates the orientation estimate, and writes one fixed 40-byte binary record to the log file. Each record contains a `uint32_t` microsecond timestamp followed by nine IEEE-754 `float32` values: pitch, roll, yaw, acceleration X/Y/Z, and gyro X/Y/Z. SdFat holds the records in its 512-byte sector cache and writes each sector to the card as it fills. The file is flushed to the filesystem every five seconds. Writing raw binary avoids CSV formatting on the Nano and substantially reduces the data size.

If an IMU read fails or returns frozen data (ten identical samples in a row), that record is skipped and the gap shows up in the timestamps. The first failure is reported over serial. If an SD write fails, the firmware prints an error and stops.

Remove the SD card only after stopping or powering down the logger so the last buffered data is not lost.

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
| Gyro X/Y/Z | degrees/s | Bias-calibrated angular rates, filtered by the MPU-6050's 98 Hz low-pass filter |

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
include/imu.h       MPU-6050 interface and data accessors (shared with the Drone's IMU driver)
src/imu.cpp         Sensor initialization, calibration, filtering, and orientation
src/main.cpp        SdFat setup, file naming, 500 Hz timer, and binary logging loop
platformio.ini      Nano, framework, monitor, and library configuration
```