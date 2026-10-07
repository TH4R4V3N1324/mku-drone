# MKU Drone

Experimental Arduino flight-controller and flight-data-logging workspace.
The repository contains the flight firmware, a standalone IMU logger, Python
utilities for serial tuning and log conversion, and 3D-printable project
models.

This is development hardware and software, not a flight-ready product. Read
the safety notes below before powering motors or ESCs.

## Projects

### [Drone](Drone/README.md)

PlatformIO firmware for an Arduino Uno. The controller:

- Reads an MPU-6050 over I2C, a PMW3901 optical flow sensor over SPI, and a
  six-channel PWM receiver.
- Runs experimental roll, pitch, yaw-rate, and hover PID control, plus optical
  flow drift hold.
- Mixes the control outputs for four ESCs in an X configuration.
- Supports serial PID tuning and Teleplot-compatible telemetry.

The project README contains the complete pinout, ESC calibration procedure,
optical flow check, control-loop details, known limitations, and tuning
instructions.

### [DataLogger](DataLogger/README.md)

PlatformIO firmware for an Arduino Nano. It calibrates an MPU-6050 and writes
fixed-size binary records to a microSD card as `FlightN.bin`. Each record is
40 bytes and contains a timestamp, pitch/roll/yaw, acceleration, and gyro
measurements.

See the project README for the wiring table, binary format, SD-card behavior,
and troubleshooting details.

### [Tools](Tools/)

- [`pid_tuner.py`](Tools/pid_tuner.py) plots live Drone telemetry and sends PID
  commands over the same serial connection.
- [`bin_to_csv.py`](Tools/bin_to_csv.py) converts a DataLogger binary flight
  log into a CSV file, including elapsed time and handling `micros()` rollover.

### [Models](Models/)

3D-printable covers, pads, propeller guards, and test-stand parts for the
hardware.

## Requirements

- [PlatformIO](https://platformio.org/), either the CLI or the VS Code
  extension.
- Python 3 for the desktop tools.
- `pyserial` and `matplotlib` for live PID tuning.

Install the Python dependencies from the repository root:

```text
python -m pip install pyserial matplotlib
```

`bin_to_csv.py` uses only the Python standard library.

## Build and upload

Each firmware project has its own `platformio.ini`. Run PlatformIO from the
project directory you want to build:

```text
cd Drone
pio run
pio run --target upload
pio device monitor --baud 115200
```

For the Nano data logger:

```text
cd DataLogger
pio run
pio run --target upload
pio device monitor --baud 115200
```

PlatformIO normally detects the upload port. If it does not, select the port
in the IDE or add an `upload_port` setting to the relevant
`platformio.ini`.

## Live PID tuning

The Drone firmware can emit Teleplot-compatible telemetry and accept PID
commands over the same serial connection. Enable `TUNE_PID` in
`Drone/src/main.cpp`, upload the firmware, and then run the tuner from the
repository root:

```text
python Tools/pid_tuner.py COM5
```

Replace `COM5` with the board's serial port. If no port is supplied, the tool
lists available serial ports and prompts you to choose one. The default baud
rate is `115200`; it can be changed with `--baud`:

```text
python Tools/pid_tuner.py COM5 --baud 115200 --window 300
```

The tuner plots the selected setpoint, measured value, and PID output while
also forwarding commands typed at its prompt:

| Command | Purpose |
| --- | --- |
| `axis r` | Select roll tuning |
| `axis p` | Select pitch tuning |
| `axis y` | Select yaw tuning |
| `axis h` | Select hover tuning |
| `axis dr` | Select roll drift hold tuning |
| `axis dp` | Select pitch drift hold tuning |
| `p <value>` | Set proportional gain |
| `i <value>` | Set integral gain |
| `d <value>` | Set derivative gain |
| `reset` | Clear controller state |
| `show` | Print the active gains |
| `quit` | Exit the tuner |

PID tuning is experimental. Keep propellers removed while changing gains or
testing the control loop.

## Working with flight logs

After stopping or powering down the DataLogger, copy a `FlightN.bin` file from
the microSD card and convert it from the repository root:

```text
python Tools/bin_to_csv.py Flight3.bin
```

This writes `Flight3.csv` beside the input file. To choose a different output
path:

```text
python Tools/bin_to_csv.py Flight3.bin analysis\flight3.csv
```

The logger flushes its buffer every five seconds. Do not remove the SD card
while the logger is running, or the final buffered records may be lost.

## Safety

The Drone firmware is experimental and has not been flight-validated.

- Remove all propellers during setup, wiring, calibration, receiver, IMU,
  ESC, PID, and failsafe testing.
- Keep motors away from people and use a secure test stand where appropriate.
- Power ESCs and motors from a suitable external supply; never power motors
  from the Arduino 5 V pin.
- Verify receiver channel order, IMU orientation, motor numbering, propeller
  direction, ESC arming behavior, and minimum ESC pulses before any powered
  test.
- Treat hover mode as experimental. It is not an arm/disarm mechanism or a
  receiver-loss failsafe.
- Run the optical flow check before flying with drift hold. A wrong sensor
  orientation makes it push the drone further in the direction it is drifting.

## Repository layout

```text
Drone/                  Arduino Uno flight-controller firmware
DataLogger/             Arduino Nano binary IMU logger
Tools/                  Python tuning and log-conversion utilities
Models/                 3D-printable hardware models
```

No project license has been specified yet.