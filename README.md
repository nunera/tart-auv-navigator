# TAMUG AUV Navigator

Python control software for a three-thruster AUV built around Blue Robotics Navigator hardware. It reads operator input and sensors, evaluates the safety policy, logs the decision, and then writes PWM through one actuator boundary.

This repository is for the current Navigator runtime. Local captures, old SD-card scripts, calibration results, and vehicle logs are intentionally excluded from Git.

> [!WARNING]
> The software has hardware-free test coverage only. It has not been verified against this vehicle's Navigator, ESCs, thrusters, wiring, sensor mounting, or BlueOS installation. Remove propellers and use an independent motor-power disconnect for every first-use or configuration-change test.

## What is here

- `auv/actuation.py` owns Navigator PWM, mixing, arming, disarming, and fault shutdown.
- `auv/safety.py` decides whether the selected manual or autonomous command may move the vehicle.
- `auv/navigation.py` contains the waypoint controller and geographic math without hardware dependencies.
- `auv/sensors.py` and `auv/hardware.py` handle NMEA GPS, LSM9DS1 heading, and ADS1115 TDS input.
- `auv/runtime.py` is the control loop. No input or sensor adapter can command a motor directly.
- `tools/` contains the isolated Navigator preflight and magnetometer capture/calibration utilities.
- `config/vehicle.example.json` is the starting point for a vehicle-specific configuration.

The default PWM mapping is physical Navigator headers 1, 2, and 3, represented in Python as channels `0`, `1`, and `2`. The horizontal motors are mixed as port and reversed-starboard; channel 2 is vertical. Confirm direction with the propellers removed before operating in water.

## Before deployment

Direct access through `bluerobotics_navigator` must not run alongside ArduPilot. In BlueOS, disable ArduPilot or select SITL for its board, then reboot the Raspberry Pi before starting this runtime.

Navigator's servo rail needs an appropriate supply. Connect ESC signal and signal ground to the relevant PWM header. Do not power a motor from the Navigator, and do not parallel BEC outputs onto the servo rail.

The native Navigator library can abort the process outside Python's exception handling. `tools/navigator_preflight.py` exists so that failure stays in a disposable process rather than the control runtime.

## Install and configure

Run these commands from the repository root on the Navigator Raspberry Pi:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

cp config/vehicle.example.json config/vehicle.json
```

Edit `config/vehicle.json` before use:

| Field | Purpose |
| --- | --- |
| `channels` | Three zero-based Navigator PWM channel numbers. |
| `frequency_hz` | ESC PWM frequency. The default is 50 Hz. |
| `speed_scale` | Global thrust limit from 0 to 1. Start below full power. |
| `horizontal_trim` | Straight-line yaw correction. It is ignored at neutral. |
| `input` | `keyboard` or `xbox`. |
| `gps_device` / `gps_baudrate` | GPS serial device and baud rate. |
| `heading_calibration` | Path to the generated magnetometer calibration JSON. |
| `sensor_to_body` | LSM9DS1-to-vehicle axis transform. Set this for the installed sensor orientation. |
| `heading_source` | `amg` for LSM9DS1 heading or `gps` for track heading. |
| `waypoints` | Latitude/longitude targets. An empty list leaves the runtime in manual-only operation. |

`Data/` stays local by design. The runtime creates CSV logs under `Data/GPSlogs/`; calibration output belongs under `Data/calibration/`.

## Bench procedure

1. Remove propellers. Keep motor power disconnected until the system is ready.
2. Check the configuration, ESC signal wiring, channel numbers, and sensor mounting.
3. Run preflight in its own process:

   ```bash
   python tools/navigator_preflight.py --channels 0,1,2
   ```

   Do not continue if it exits nonzero.

4. Start the runtime without PWM enabled:

   ```bash
   python -m auv --config config/vehicle.json
   ```

5. After confirming expected startup, run the armed session:

   ```bash
   python -m auv --config config/vehicle.json --arm
   ```

`Ctrl-C` and `SIGTERM` command neutral output, wait for the neutral dwell when armed, disable PWM, and close the run log. A process crash, native abort, or power loss still requires a physical motor-power disconnect.

## Controls and motion policy

Keyboard control uses `W/S` for forward/reverse, `A/D` for yaw, `R/F` for vertical movement, `X` for neutral, `M` to request autonomous mode, and space to latch an emergency stop. The keyboard path requires repeated events to remain fresh; if it goes stale, the runtime commands neutral PWM.

Xbox control uses the left stick for forward/yaw, triggers for vertical motion, START to request autonomy, and BACK for emergency stop.

Autonomy needs fresh, valid GPS and heading data. Invalid or void NMEA fixes clear the current position; `(0, 0)` is never treated as a valid location. If AMG heading is unavailable, autonomous motion is neutral. With GPS track heading, the controller may move straight ahead at no more than 10% while waiting for a second distinct GPS fix. An emergency stop, sensor-reader exception, or log-write failure faults the actuator; restart and re-arm only after correcting the cause.

## Magnetometer calibration

With propellers removed, rotate the installed vehicle through its full range of orientations:

```bash
python tools/capture_magnetometer.py Data/calibration/magnetometer-samples.csv --seconds 60
python tools/calibrate_magnetometer.py Data/calibration/magnetometer-samples.csv \
  --output Data/calibration/magnetometer.json
```

The calibration utility estimates hard-iron offset and per-axis soft-iron scale. It cannot infer how the sensor board is mounted. Set `sensor_to_body` in the vehicle config, then bench-check heading against known north and east directions before autonomous operation.

## Development checks

The suite uses a fake Navigator backend, so it covers runtime behavior without PWM hardware:

```bash
PYTHONPATH=. python -m py_compile auv/*.py tools/*.py
PYTHONPATH=. python -m pytest tests -q
```

These checks cover PWM neutral/disarm ordering, fault latching, mixing, NMEA fix invalidation, GPS-heading acquisition limits, navigation behavior, calibration persistence, configuration loading, and runtime integration. They do not validate physical hardware.

## References

- [Blue Robotics navigator-lib](https://github.com/bluerobotics/navigator-lib/blob/master/README.md)
- [bluerobotics-navigator 0.2.0](https://pypi.org/project/bluerobotics-navigator/0.2.0/)
