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
| Roll | 11 |
| Pitch | 10 |
| Yaw | 8 |
| AUX1 | 12 |
| AUX2 | 13 |

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

## ESC calibration

ESC calibration is built into the flight-controller firmware and is selected with the `CALIBRATE_ESCS` flag in `src/main.cpp`. When enabled, the firmware sends the same calibration signal to all four ESC outputs (Arduino pins `4-7`): maximum throttle for 15 seconds, followed by minimum throttle for 10 seconds. It then stops and does not enter the normal flight loop.

With propellers removed:

1. Set `CALIBRATE_ESCS` to `1` in `src/main.cpp`.
2. Upload the firmware with the ESCs ready to receive power, but with propellers removed.
3. Power the Arduino and ESCs when prompted by the serial messages, then listen for the ESC confirmation tones.
4. Wait through the 15-second maximum-throttle phase and the 10-second minimum-throttle phase.
5. Power-cycle the ESCs after the firmware reports that calibration pulses are complete.
6. Set `CALIBRATE_ESCS` back to `0` and upload the firmware again before flying.

The calibration routine prints its progress at `115200` baud and intentionally halts after sending both pulse ranges. Keep the motors disconnected or the propellers removed, and use a common ground between the Arduino and ESC signal ground.

## IMU level calibration

"Level" is defined by accelerometer offsets that depend on how the IMU is mounted, so they are measured once and stored in EEPROM. The gyro bias drifts with temperature and is still measured at every boot.

1. Set `CALIBRATE_IMU` to `1` in `src/main.cpp` and upload.
2. Open the serial monitor at `115200` baud.
3. Put a spirit level on the frame (not the table) and shim the drone until the frame is level.
4. Follow the prompts: five rounds of about 6 seconds each. Between rounds, pick the drone up, tilt it around, and set it back down level, then send any key to start the next round. Rounds where the drone moved are repeated.
5. The firmware prints each round, the average, and the spread between rounds. It only saves to EEPROM if all rounds agree within `0.3` degrees. A larger spread usually means a soft surface or a loose IMU mount.
6. Set `CALIBRATE_IMU` back to `0` and upload again before flying.

If it still drifts consistently in hover, adjust `ROLL_TRIM_DEG` and `PITCH_TRIM_DEG` in `src/main.cpp` in steps of about `0.5` degrees. Without position sensing (optical flow or GPS) some drift with air movement is expected.

## Firmware behavior

At startup the firmware initializes serial communication at `115200` baud, wakes the MPU-6050, measures the gyro bias (retrying until the drone is still), loads the level calibration from EEPROM, and seeds the angle estimate from the accelerometer. If no valid calibration is stored, it measures level at boot instead and prints a warning. It then initializes the receiver, starts the ESC output at minimum throttle, and waits two seconds. Arming requires a live receiver link and the throttle seen at minimum first.

The default loop runs the experimental PID control path. It:

1. Reads the IMU and receiver.
2. Estimates vertical velocity from filtered, tilt-compensated accelerometer data.
3. Uses the throttle stick directly and stops all motors when throttle is at or below 5%, unless hover mode is active.
4. Converts receiver commands to roll, pitch, and yaw-rate setpoints.
5. Computes cascaded roll and pitch corrections (angle loop feeding a gyro rate loop), the yaw rate correction, and, when active, the hover PID output.
6. Mixes the commands and writes normalized `0.0-1.0` motor speeds.

### Hover mode

Hover mode is enabled with `AUX1` and uses `AUX2` to command vertical movement:

- Set `AUX1` high while the drone is airborne, the throttle is above 20%, and estimated vertical speed remains below `0.3 m/s` for `0.5` seconds. The firmware captures the filtered throttle value as the hover throttle and resets the hover controller for a bumpless transition.
- While active, `AUX2` commands a climb or descent rate from approximately `-1` to `+1 m/s`. A `0.1 m/s` deadband prevents small stick movements from causing corrections.
- The hover controller adjusts throttle around the captured value, limits the correction to a band around that value, applies a slew rate, and compensates for roll and pitch tilt. Throttle remains constrained between the hover floor and `MAX_THROTTLE`.
- Set `AUX1` low to leave hover mode and return to direct throttle control. The transition ramps from the current hover throttle toward the stick command; moving the stick to the low-throttle cutoff still stops the motors immediately. Hover mode is also not an arm/disarm or receiver-loss failsafe.

Hover mode is experimental. The vertical-velocity estimate is derived from integrated accelerometer data with filtering and leakage, so drift and acceleration bias can affect altitude behavior. Test it with propellers removed first and be ready to disable `AUX1`.

Set `TUNE_PID` to `1` in `src/main.cpp` to enable live PID command handling and telemetry. The `test()` function is available for receiver and mixer checks without PID control; enable it in `loop()` only with propellers removed.

## Live PID tuning

The firmware emits Teleplot-compatible telemetry for the selected axis. The Python helper in `../Tools/pid_tuner.py` plots that telemetry and sends gain changes over the same serial connection.

From the repository root:

```text
python -m pip install pyserial matplotlib
python Tools/pid_tuner.py COM5
```

Use the commands `axis rr` or `axis pr` to select the roll or pitch rate (inner) loop, `axis r` or `axis p` for the roll or pitch angle (outer) loop, and `axis y` or `axis h` for yaw rate or hover tuning. Tune the rate loops first. Use `p <value>`, `i <value>`, and `d <value>` to change gains, `reset` to clear controller state, and `show` to print the active gains. Replace `COM5` with the board's serial port.

## Project structure

```text
include/                 Public firmware headers
  imu.h                   MPU-6050 and orientation interface
  escOutput.h             Timer1 ESC output interface
  motorMixer.h            Quad-X mixer interface
  pid.h                   PID controller
  receiver.h              PWM receiver interface
  serialTuner.h           Runtime PID tuning commands
  calibrationStore.h      EEPROM storage for the IMU level calibration
src/                     Firmware implementation
  main.cpp                Setup, control loop, telemetry, calibration modes, and test loop
  imu.cpp                 MPU-6050 reading, calibration, and complementary filter
  calibrationStore.cpp    EEPROM load/save with checksum and validation
  escOutput.cpp           Timer1 ESC pulse generation
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