# Navigator hardware checklist

The authoritative deployment, calibration, control, and fault procedures are in [`README.md`](README.md).

Before `python -m auv --arm`:

1. Remove propellers and prepare a physical motor-power disconnect.
2. Disable ArduPilot or set its BlueOS board to SITL; reboot. Direct Navigator PWM access and ArduPilot cannot share the Navigator.
3. Verify the zero-based channel mapping: physical PWM 1/2/3 are Python channels `0`/`1`/`2`.
4. Run `python tools/navigator_preflight.py --channels 0,1,2` in its own process. An exit other than zero blocks operation.
5. Run `python -m auv --config config/vehicle.json` without `--arm`; only after observing normal startup may the explicit `--arm` invocation be used.

Navigator's servo rail needs its own appropriate supply. ESC signal ground must reach the Navigator PWM header; do not power motors from the Navigator, and never parallel multiple ESC BEC feeds onto the servo rail.
