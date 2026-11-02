# MKU Drone

Arduino flight-controller and flight-data-logging workspace.
The repository contains the flight firmware, a standalone IMU logger, Python
and MATLAB utilities for serial tuning, log conversion, and analysis, and
3D-printable project models.

The flight controller, hover mode, and drift hold have been flight-validated.
Read the safety notes below before powering motors or ESCs.

## Projects

### [Drone](Drone/README.md)

PlatformIO firmware for an Arduino Uno. The controller:

- Reads an MPU-6050 over I2C, a PMW3901 optical flow sensor over SPI, and a
  six-channel PWM receiver.
- Runs a fixed 250 Hz loop with cascaded angle/rate roll and pitch control,
  yaw-rate control, hover control, and optical flow drift hold.
- Mixes the control outputs for four ESCs in an X configuration.
- Arms only from a low throttle with the drone level, and cuts the motors on
  receiver loss, IMU failure, or a crash tilt.
- Supports serial PID tuning, Teleplot-compatible telemetry, and loop-timing
  measurement.

The project README contains the complete pinout, ESC and IMU calibration
procedures, optical flow check, arming and safety behavior, control-loop
details, known limitations, and tuning instructions.

### [DataLogger](DataLogger/README.md)

PlatformIO firmware for an Arduino Nano. It calibrates an MPU-6050 and writes
fixed-size binary records at 500 Hz to a microSD card as `FlightN.bin`. Each
record is 40 bytes and contains a timestamp, pitch/roll/yaw, acceleration, and gyro
measurements.

See the project README for the wiring table, binary format, SD-card behavior,
and troubleshooting details.

### [Tools](Tools/)

- [`pid_tuner.py`](Tools/pid_tuner.py) plots live Drone telemetry and sends PID
  commands over the same serial connection.
- [`bin_to_csv.py`](Tools/bin_to_csv.py) converts a DataLogger binary flight
  log into a CSV file, including elapsed time and handling `micros()` rollover.
- [`CSV_analyser.py`](Tools/CSV_analyser.py) prints the mean, RMS, standard
  deviation, and peak values of every numeric column in a converted CSV log.
- [`FFT_Anaylsis_Code.m`](Tools/FFT_Anaylsis_Code.m) is a MATLAB script that
  plots the single-sided FFT amplitude spectrum of one column for every CSV in
  a folder, for example to find motor vibration frequencies in `accel_z`.

### [Models](Models/)

Fusion 360 (`.f3d`) sources and printable `.stl` files:

- `Models/Drone/`: Uno cover and lid, battery cap, IMU pad, optical flow
  mount, leg reinforcement, and propeller guards.
- `Models/TestStand/`: pivot arm, block, bushes, and stiffening bar for the
  single-axis test rig.

## Requirements

- [PlatformIO](https://platformio.org/), either the CLI or the VS Code
  extension.
- Python 3 for the desktop tools.
- `pyserial` and `matplotlib` for live PID tuning.

Install the Python dependencies from the repository root:

```text
python -m pip install pyserial matplotlib
```

`bin_to_csv.py` uses only the Python standard library. `CSV_analyser.py`
needs `pandas` and `numpy`:

```text
python -m pip install pandas numpy
```

`FFT_Anaylsis_Code.m` needs MATLAB.

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

The tuner plots the selected setpoint, measured value, PID output, and
integral term while also forwarding commands typed at its prompt:

| Command | Purpose |
| --- | --- |
| `axis rr` | Select roll rate (inner loop) tuning |
| `axis r` | Select roll angle (outer loop) tuning |
| `axis pr` | Select pitch rate (inner loop) tuning |
| `axis p` | Select pitch angle (outer loop) tuning |
| `axis y` | Select yaw rate tuning |
| `axis h` | Select hover tuning |
| `axis dr` | Select roll drift hold tuning |
| `axis dp` | Select pitch drift hold tuning |
| `p <value>` | Set proportional gain |
| `i <value>` | Set integral gain |
| `d <value>` | Set derivative gain |
| `reset` | Clear controller state |
| `show` | Print the active gains |
| `quit` | Exit the tuner |

The roll rate loop is selected at startup. Tune the rate loops before the
angle loops, and drift hold last. Gain changes are not saved to the board;
copy the final values into `Drone/src/main.cpp`.

Keep propellers removed while changing gains or testing the control loop on
the bench.

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

The logger flushes the file every five seconds. Do not remove the SD card
while the logger is running, or the final buffered records may be lost.

To print summary statistics for a converted log, set `csv_file` at the top of
`Tools/CSV_analyser.py` to the CSV's path (relative to the directory you run
it from) and run:

```text
python Tools/CSV_analyser.py
```

For a frequency analysis, set `folderList` (the folder of CSV files) and
`signalCol` at the top of `Tools/FFT_Anaylsis_Code.m` and run it in MATLAB.
`timeCol = 2` and `signalCol = 8` select the `time_s` and `accel_z` columns
written by `bin_to_csv.py`.

## Safety

The Drone flight controller, hover mode, and drift hold have been
flight-validated, but this remains hobby hardware and software under active
development.

- Remove all propellers during setup, wiring, calibration, receiver, IMU,
  ESC, PID, and failsafe testing.
- Keep motors away from people and use a secure test stand where appropriate.
- Power ESCs and motors from a suitable external supply; never power motors
  from the Arduino 5 V pin.
- Verify receiver channel order, IMU orientation, motor numbering, propeller
  direction, ESC arming behavior, and minimum ESC pulses before any powered
  test.
- Practise hover mode at low height over open ground first, with a thumb
  on the throttle stick: lowering it to idle disarms immediately, even in
  hover.
- Receiver loss cuts the motors; there is no controlled-descent failsafe.
  Test arming, disarming, receiver loss, and the crash cutoff with propellers
  removed.
- Run the optical flow check before flying with drift hold. A wrong sensor
  orientation makes it push the drone further in the direction it is drifting.

## Repository layout

```text
Drone/                  Arduino Uno flight-controller firmware
DataLogger/             Arduino Nano binary IMU logger
Tools/                  Python and MATLAB tuning, log-conversion, and analysis utilities
Models/                 3D-printable hardware models
```

No project license has been specified yet.