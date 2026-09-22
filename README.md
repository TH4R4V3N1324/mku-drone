# Drone Flight Controller

An experimental quadcopter flight-controller firmware project for an Arduino Uno. The firmware reads an MPU-6050 IMU and four receiver channels, then drives four ESCs through a simple X-configuration motor mixer.

This is a development prototype. Do not connect propellers or operate the motors near people while testing.

## Hardware

- Arduino Uno
- MPU-6050 IMU at I2C address `0x68`
- Four ESCs and brushless motors
- Receiver providing four PWM channels:
  - Throttle: pin `10`
  - Roll: pin `8`
  - Pitch: pin `9`
  - Yaw: pin `11`

### Motor outputs

| Motor | Arduino pin | Mixer position |
| --- | ---: | --- |
| Motor 1 | 4 | Front left |
| Motor 2 | 5 | Front right |
| Motor 3 | 6 | Rear right |
| Motor 4 | 7 | Rear left |

Connect the receiver and IMU grounds to the Arduino ground. Power the motors and ESCs with an appropriate external supply; do not power them from the Arduino 5 V pin.

## Software requirements

- [PlatformIO](https://platformio.org/)
- Arduino framework for the Uno

Dependencies are declared in `platformio.ini`:

- `Servo`
- `SD`

The current source uses `Servo` and `Wire`; `SD` is declared for planned or future functionality.

## Build and upload

From the project directory:

```text
pio run
pio run --target upload
pio device monitor --baud 9600
```

The upload port may need to be selected in PlatformIO or configured in `platformio.ini`.

## Current operation

On startup, the firmware:

1. Starts serial communication at `9600` baud.
2. Initializes the I2C bus and wakes the MPU-6050.
3. Runs a short IMU calibration routine.
4. Initializes the receiver inputs.

The default `loop()` currently runs `test()`, not the stabilized flight-control loop. `test()` reads the receiver and sends the throttle value to all four motors while printing receiver data over serial.

To use the experimental control path, change `loop()` in `src/main.cpp` so it calls `run()` instead of `test()`:

```cpp
void loop() {
  run();
}
```

The control path reads IMU orientation and gyro data, converts receiver inputs into roll, pitch, and yaw setpoints, applies PID controllers, and sends the results to the motor mixer.

## Project structure

```text
include/                 Public headers for the firmware components
  imu.h                   MPU-6050 data and orientation interface
  motor.h                 ESC output interface
  motorMixer.h            Quad-X motor mixing interface
  pid.h                   PID controller
  receiver.h              PWM receiver input interface
src/                     Firmware implementation
  main.cpp                Setup, control loop, and test loop
  imu.cpp                MPU-6050 reading and complementary filter
  motor.cpp              ESC pulse generation
  motorMixer.cpp         Quad-X mixing
  receiver.cpp           Receiver pulse decoding
platformio.ini            PlatformIO environment configuration
```

## Development notes

- Receiver pulses are accepted in the approximate `900-2100` microsecond range.
- Throttle is normalized to `0.0-1.0`; roll, pitch, and yaw are normalized to `-1.0-1.0`.
- IMU roll and pitch use a complementary filter. Yaw is integrated from the gyroscope and will drift without a heading reference.
- The receiver reader uses blocking `pulseIn()` calls with a 25 ms timeout, which limits control-loop responsiveness.
- Before enabling `run()`, verify the motor numbering, propeller direction, receiver channel order, sensor orientation, ESC arming behavior, and controller gains.
- The mixer currently constrains intermediate values to `0-100`, while `Motor::setSpeed()` expects `0-1`. This output scaling should be corrected and bench-tested before using the stabilized path.

## TODO Before Flight

- [ ] Replace the blocking `pulseIn()` receiver reads with interrupt-based capture, or confirm that the control loop stays fast enough.
- [ ] Normalize and limit PID outputs before sending them to the mixer. The mixer and `Motor::setSpeed()` must use the same `0.0-1.0` scale.
- [ ] Add arming/disarming and a receiver-loss failsafe that stops all motors.
- [ ] Verify IMU axis directions, motor order, propeller direction, and ESC minimum pulse with propellers removed.
- [ ] Test receiver unplugging, IMU failure, disarming, and power cycling with propellers removed.

EEPROM is not required for the fixed pin setup. The reference sketches use it for receiver calibration, axis mapping, and gyro configuration, but this project can keep those values in code. Add receiver endpoint calibration later if the channels are not close to `1000/1500/2000` microseconds.

## License

No license has been specified for this project yet.