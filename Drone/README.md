# Drone Flight Controller

Experimental quadcopter flight-controller firmware for an Arduino Uno. The firmware reads an MPU-6050 IMU, a PMW3901 optical flow sensor, and six receiver channels, applies PID corrections, and drives four ESCs through an X-configuration motor mixer.

This is a development prototype, not flight-ready software. Keep propellers removed during development and bench testing, and keep motors away from people.

## Hardware and wiring

- Arduino Uno
- MPU-6050 on I2C at address `0x68`
- PMW3901 optical flow breakout (Pimoroni) on SPI, facing down
- Four ESCs and brushless motors
- Receiver with six PWM channels

### Receiver inputs

| Channel | Arduino pin |
| --- | ---: |
| Throttle | A0 |
| Roll | A1 |
| Pitch | A2 |
| Yaw | A3 |
| AUX1 | 2 |
| AUX2 | 3 |

The receiver is decoded with pin-change interrupts and can use any pins except `0`/`1` (Serial). Pins `10-13` are kept free for the flow sensor's SPI bus.

### Optical flow sensor

| PMW3901 pin | Arduino pin |
| --- | ---: |
| CS | 10 |
| MOSI | 11 |
| MISO | 12 |
| SCK | 13 |
| 3-5V | 5V |
| GND | GND |

`INT` is not used. CS must stay on pin `10`: if that pin were an input pulled low, the Uno would drop out of SPI master mode.

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

The firmware uses the Arduino framework's built-in `Wire` library for the IMU and `SPI` library for the flow sensor. No additional PlatformIO libraries are currently required.

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

If it still drifts consistently in hover, adjust `ROLL_TRIM_DEG` and `PITCH_TRIM_DEG` in `src/main.cpp` in steps of about `0.5` degrees. Drift hold (below) corrects remaining drift once the flow sensor is set up, but good trim keeps its corrections small.

## Optical flow check

The flow sensor's orientation and scale depend on how it is mounted, and a wrong sign makes drift hold push the drone the way it is already drifting. Set them with the bench check before flying.

1. Set `CHECK_FLOW` to `1` in `src/main.cpp` and upload. The ESCs receive no signal in this mode.
2. Open Teleplot on the serial port at `115200` baud and hold the drone 20-50 cm over a textured, well-lit floor.
3. Tilt it left and right without sliding it. All flow then comes from rotation, so `flowRight` should overlap `rotRight` and `velRight` should stay near zero.
   - Mirrored curves: set `FLOW_RIGHT_SIGN` to `-1`.
   - Movement shows on `flowForward` instead: set `FLOW_SWAP_XY` to `true`.
   - Right shape, wrong size: multiply `FLOW_RAD_PER_COUNT` by `rotRight / flowRight`.
4. Repeat for pitch with `flowForward`, `rotForward`, and `FLOW_FORWARD_SIGN`.
5. Slide the drone right without tilting it: `velRight` should go positive. Slide it forward: `velForward` should go positive. "Right" and "forward" are the directions positive roll and pitch stick fly.
6. Set `CHECK_FLOW` back to `0` and upload again before flying.

`quality` shows how many surface features the sensor is tracking. If it stays low, the surface is too plain or too dark.

## Firmware behavior

At startup the firmware initializes serial communication at `115200` baud, wakes the MPU-6050, measures the gyro bias (retrying until the drone is still), loads the level calibration from EEPROM, and seeds the angle estimate from the accelerometer. If no valid calibration is stored, it measures level at boot instead and prints a warning. It then checks for the optical flow sensor (if it is missing, the firmware prints a warning and flies without drift hold), initializes the receiver, starts the ESC output at minimum throttle, and waits two seconds. Arming requires a live receiver link and the throttle seen at minimum first.

The default loop runs the experimental PID control path. It:

1. Reads the IMU and receiver.
2. Estimates vertical velocity from filtered, tilt-compensated accelerometer data.
3. Uses the throttle stick directly and stops all motors when throttle is at or below 5%, unless hover mode is active.
4. Converts receiver commands to roll, pitch, and yaw-rate setpoints, adding the drift hold correction when it is active.
5. Computes cascaded roll and pitch corrections (angle loop feeding a gyro rate loop), the yaw rate correction, and, when active, the hover PID output.
6. Mixes the commands and writes normalized `0.0-1.0` motor speeds.

### Hover mode

Hover mode is enabled with `AUX1` and uses `AUX2` to command vertical movement:

- Set `AUX1` high while the drone is airborne, the throttle is above 20%, and estimated vertical speed remains below `0.3 m/s` for `0.5` seconds. The firmware captures the filtered throttle value as the hover throttle and resets the hover controller for a bumpless transition.
- While active, `AUX2` commands a climb or descent rate from approximately `-1` to `+1 m/s`. A `0.1 m/s` deadband prevents small stick movements from causing corrections.
- The hover controller adjusts throttle around the captured value, limits the correction to a band around that value, applies a slew rate, and compensates for roll and pitch tilt. Throttle remains constrained between the hover floor and `MAX_THROTTLE`.
- Set `AUX1` low to leave hover mode and return to direct throttle control. The transition ramps from the current hover throttle toward the stick command; moving the stick to the low-throttle cutoff still stops the motors immediately. Hover mode is also not an arm/disarm or receiver-loss failsafe.

Hover mode is experimental. The vertical-velocity estimate is derived from integrated accelerometer data with filtering and leakage, so drift and acceleration bias can affect altitude behavior. Test it with propellers removed first and be ready to disable `AUX1`.

### Drift hold

Drift hold uses the optical flow sensor to stop the drone sliding sideways. It is active whenever the drone is airborne: in hover mode, or when the filtered throttle stick is above `DRIFT_MIN_THROTTLE` (`0.2`).

- Every fifth loop (50 Hz) the firmware reads the sensor, removes the apparent ground motion caused by tilting (measured by the gyro over the same interval), and low-pass filters the result at `5 Hz`.
- The result is ground speed divided by height, in rad/s. No rangefinder is needed: the loop drives it to zero, which means zero drift at any height.
- Two PI controllers turn it into roll and pitch angle offsets of up to `DRIFT_CORRECTION_LIMIT` (`8` degrees).
- Moving the roll or pitch stick turns drift hold off on that axis so the pilot can reposition; it resumes when the stick is centred.
- Corrections drop to zero when the sensor reports poor surface quality, such as below about 8 cm or over plain or dark surfaces.

Because the signal is speed divided by height, the loop is stronger the lower the drone flies. Tune it at the lowest height you will fly at, and watch for rocking close to the ground during takeoff and landing. If that happens, raise `DRIFT_MIN_THROTTLE` or lower the gain. Above about 3 m the correction becomes very weak.

Set `TUNE_PID` to `1` in `src/main.cpp` to enable live PID command handling and telemetry. The `test()` function is available for receiver and mixer checks without PID control; enable it in `loop()` only with propellers removed.

## Live PID tuning

The firmware emits Teleplot-compatible telemetry for the selected axis. The Python helper in `../Tools/pid_tuner.py` plots that telemetry and sends gain changes over the same serial connection.

From the repository root:

```text
python -m pip install pyserial matplotlib
python Tools/pid_tuner.py COM5
```

Use the commands `axis rr` or `axis pr` to select the roll or pitch rate (inner) loop, `axis r` or `axis p` for the roll or pitch angle (outer) loop, `axis y` or `axis h` for yaw rate or hover tuning, and `axis dr` or `axis dp` for the roll or pitch drift hold. Tune the rate loops first and drift hold last. The drift gains start at `kp 8` with no integral; raise `p` if it drifts slowly, lower it if it rocks, then add a small `i` to hold against wind. Use `p <value>`, `i <value>`, and `d <value>` to change gains, `reset` to clear controller state, and `show` to print the active gains. Replace `COM5` with the board's serial port.

## Project structure

```text
include/                 Public firmware headers
  imu.h                   MPU-6050 and orientation interface
  escOutput.h             Timer1 ESC output interface
  motorMixer.h            Quad-X mixer interface
  pid.h                   PID controller
  receiver.h              PWM receiver interface
  opticalFlow.h           PMW3901 optical flow interface
  serialTuner.h           Runtime PID tuning commands
  calibrationStore.h      EEPROM storage for the IMU level calibration
src/                     Firmware implementation
  main.cpp                Setup, control loop, telemetry, calibration modes, and test loop
  imu.cpp                 MPU-6050 reading, calibration, and complementary filter
  calibrationStore.cpp    EEPROM load/save with checksum and validation
  escOutput.cpp           Timer1 ESC pulse generation
  motorMixer.cpp          Quad-X mixing
  receiver.cpp            Receiver pulse decoding
  opticalFlow.cpp         PMW3901 SPI driver, start-up sequence, and motion burst reads
platformio.ini            PlatformIO configuration
```

## Known limitations and pre-flight work

- There is no explicit arm/disarm state or receiver-loss failsafe beyond invalid channels becoming zero.
- Yaw is integrated from the gyro and will drift without a heading reference.
- The PID gains are experimental and the controller has not been flight-validated.
- The optical flow driver and drift hold have not been tested on hardware. The flow scale `FLOW_RAD_PER_COUNT` is taken from ArduPilot's PMW3901 driver and must be checked with `CHECK_FLOW`.
- Drift hold cannot be switched off in flight.
- Verify receiver channel order, IMU orientation, motor numbering, propeller direction, ESC arming behavior, and minimum ESC pulses with propellers removed.
- Test receiver loss, IMU failure, disarming, and power cycling with propellers removed.

## License

No license has been specified for this project yet.