# MKU Flight Controller Workspace

This repository contains an experimental Arduino flight-controller project, a PlatformIO starter project for future data logging, and a desktop utility for tuning PID gains over serial.

## Projects

### `Drone`

The active prototype flight controller targets an Arduino Uno. It reads an MPU-6050 and a four-channel PWM receiver, runs roll/pitch/yaw PID control, and drives four ESC outputs using a quad-X mixer.

See [Drone/README.md](Drone/README.md) for wiring, firmware behavior, safety notes, and PID-tuning instructions.

### `DataLogger`

PlatformIO project targeting an Arduino Nano with the Arduino framework. It currently contains the default starter sketch and is reserved for future sensor or flight-data logging.

### `Tools`

`Tools/pid_tuner.py` is a Python serial utility that plots Teleplot-compatible telemetry and sends runtime PID commands to the Drone firmware.

## Requirements

- [PlatformIO](https://platformio.org/), through the CLI or VS Code extension
- Python 3 for the tuning utility
- `pyserial` and `matplotlib` for live PID tuning

Install the Python dependencies from the repository root:

```text
python -m pip install pyserial matplotlib
```

## Build the firmware

Each firmware project has its own `platformio.ini`. Run PlatformIO from the corresponding project directory:

```text
cd Drone
pio run
pio run --target upload
pio device monitor --baud 115200
```

For the Nano starter project:

```text
cd DataLogger
pio run
pio run --target upload
```

Choose the correct upload port in PlatformIO when it is not detected automatically. The Drone firmware uses `115200` baud for serial output.

## Safety

The Drone project is experimental and is not flight-ready. Remove propellers for all setup, wiring, receiver, IMU, ESC, PID, and failsafe tests. Use an appropriate external power source for the ESCs and motors, and never power the motors from an Arduino 5 V pin.

## Repository layout

```text
Drone/                  Arduino Uno flight-controller firmware
DataLogger/             Arduino Nano logging project starter
Tools/pid_tuner.py      Live serial PID tuner and plotter
Models/                 Reserved for models and supporting assets
```

No project license has been specified yet.