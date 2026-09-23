# Drone Flight Controller

Experimental quadcopter flight-controller firmware for an Arduino Uno. The firmware reads an MPU-6050 IMU and four receiver channels, applies PID corrections, and drives four ESCs through an X-configuration motor mixer.

This is a development prototype, not flight-ready software. Keep propellers removed during development and bench testing, and keep motors away from people.

## Hardware and wiring

- Arduino Uno
- MPU-6050 on I2C at address `0x68`
- Four ESCs and brushless motors
- Receiver with four PWM channels

### Receiver inputs

| Channel | Arduino pin |
| --- | ---: |
| Throttle | 9 |
| Roll | 8 |
| Pitch | 10 |
| Yaw | 11 |

### Motor outputs

| Motor | Arduino pin | Mixer position |
| --- | ---: | --- |
| Motor 1 | 4 | Front left |
| Motor 2 | 5 | Front right |
| Motor 3 | 6 | Rear right |
| Motor 4 | 7 | Rear left |

Connect the receiver and IMU grounds to Arduino ground. Power the ESCs and motors from an appropriate external supply; never power them from the Arduino 5 V pin.

## Requirements

- [PlatformIO](https://platformio.org/)
- Arduino framework for the Uno

The firmware uses the Arduino framework and its built-in `Wire` library for the IMU's I2C connection. No additional PlatformIO libraries are currently required.

## Build and upload

Run these commands from the `Drone` directory:

```text
pio run
pio run --target upload
pio device monitor --baud 115200
```

Select the upload port in PlatformIO or add an `upload_port` setting to `platformio.ini` when automatic port detection does not find the board.

## Firmware behavior

At startup the firmware initializes serial communication at `115200` baud, wakes the MPU-6050, averages 100 stationary gyro samples for calibration, initializes the receiver, stops all motors, and waits two seconds.

The default loop runs the experimental PID control path. It:

1. Reads the IMU and receiver.
2. Stops all motors when throttle is at or below 5%.
3. Converts receiver commands to roll, pitch, and yaw-rate setpoints.
4. Computes roll, pitch, and yaw PID outputs.
5. Mixes the commands and writes normalized `0.0-1.0` motor speeds.

Set `TUNE_PID` to `1` in `src/main.cpp` to enable live PID command handling and telemetry. The `test()` function is available for receiver and mixer checks without PID control; enable it in `loop()` only with propellers removed.

## Live PID tuning

The firmware emits Teleplot-compatible telemetry for the selected axis. The Python helper in `../Tools/pid_tuner.py` plots that telemetry and sends gain changes over the same serial connection.

From the repository root:

```text
python -m pip install pyserial matplotlib
python Tools/pid_tuner.py COM5
```

Use the commands `axis r`, `axis p`, or `axis y` to select an axis, `p <value>`, `i <value>`, and `d <value>` to change gains, `reset` to clear controller state, and `show` to print the active gains. Replace `COM5` with the board's serial port.

## Project structure

```text
include/                 Public firmware headers
  imu.h                   MPU-6050 and orientation interface
  motor.h                 ESC output interface
  motorMixer.h            Quad-X mixer interface
  pid.h                   PID controller
  receiver.h              PWM receiver interface
  serialTuner.h           Runtime PID tuning commands
src/                     Firmware implementation
  main.cpp                Setup, control loop, telemetry, and test loop
  imu.cpp                 MPU-6050 reading and complementary filter
  motor.cpp               ESC pulse generation
  motorMixer.cpp          Quad-X mixing
  receiver.cpp            Receiver pulse decoding
platformio.ini            PlatformIO configuration
```

## Known limitations and pre-flight work

- There is no explicit arm/disarm state or receiver-loss failsafe beyond invalid channels becoming zero.
- Yaw is integrated from the gyro and will drift without a heading reference.
- The PID gains are experimental and the controller has not been flight-validated.
- Verify receiver channel order, IMU orientation, motor numbering, propeller direction, ESC arming behavior, and minimum ESC pulses with propellers removed.
- Test receiver loss, IMU failure, disarming, and power cycling with propellers removed.

## License

No license has been specified for this project yet.