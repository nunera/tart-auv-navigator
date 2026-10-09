from pathlib import Path

from auv.actuation import ActuatorState, NavigatorActuator
from auv.input import SharedManualInput
from auv.logging import RunLogger
from auv.models import HeadingSample, MotionCommand, PositionSample
from auv.navigation import NavigationController, Waypoint
from auv.runtime import ControlRuntime
from auv.safety import SafetyController


class FakePwm:
    def __init__(self):
        self.calls = []

    def initialize(self):
        self.calls.append(("initialize",))

    def set_enabled(self, enabled):
        self.calls.append(("enabled", enabled))

    def set_frequency_hz(self, frequency):
        self.calls.append(("frequency", frequency))

    def set_duty_cycle(self, channel, duty):
        self.calls.append(("duty", channel, duty))


class FixedPosition:
    def read(self, now):
        return PositionSample(29.3136, -94.8170, now, True)


class FixedHeading:
    def __init__(self, value=0):
        self.value = value

    def read(self, now):
        return HeadingSample(self.value, now, True)


class SequencePosition:
    def __init__(self, positions):
        self.positions = iter(positions)

    def read(self, now):
        latitude, longitude = next(self.positions)
        return PositionSample(latitude, longitude, now, True)


class FailingLogger:
    def open(self):
        pass

    def record(self, **kwargs):
        raise OSError("disk full")

    def close(self):
        pass


def make_runtime(tmp_path, *, logger=None):
    backend = FakePwm()
    actuator = NavigatorActuator(backend, sleep_fn=lambda _: None)
    manual = SharedManualInput()
    runtime = ControlRuntime(
        actuator=actuator,
        manual_input=manual,
        position_reader=FixedPosition(),
        heading_reader=FixedHeading(),
        navigation=NavigationController([Waypoint(29.3146, -94.8170)]),
        safety=SafetyController(),
        logger=logger or RunLogger(Path(tmp_path) / "run.csv"),
        clock=lambda: 10.0,
    )
    runtime.initialize(arm=True)
    return runtime, manual, backend


def test_manual_command_is_single_runtime_owned_pwm_path(tmp_path):
    runtime, manual, backend = make_runtime(tmp_path)
    manual.publish(forward=0.5)

    command = runtime.step()

    assert command.forward == 0.5
    assert runtime.actuator.state is ActuatorState.ARMED
    assert any(call[0] == "duty" for call in backend.calls)
    runtime.shutdown()


def test_logging_error_latches_actuator_fault(tmp_path):
    runtime, _, _ = make_runtime(tmp_path, logger=FailingLogger())

    try:
        runtime.step()
    except RuntimeError as error:
        assert "logging failed" in str(error)
    else:
        raise AssertionError("runtime continued after telemetry persistence failed")

    assert runtime.actuator.state is ActuatorState.FAULT


def test_gps_heading_autonomy_creeps_only_until_track_heading_is_available(tmp_path):
    backend = FakePwm()
    actuator = NavigatorActuator(backend, sleep_fn=lambda _: None)
    manual = SharedManualInput()
    runtime = ControlRuntime(
        actuator=actuator,
        manual_input=manual,
        position_reader=SequencePosition([(29.3136, -94.8170), (29.3137, -94.8170)]),
        heading_reader=FixedHeading(),
        navigation=NavigationController(
            [Waypoint(29.3146, -94.8170)],
            heading_source="gps",
            max_heading_acquisition_forward=0.1,
        ),
        safety=SafetyController(),
        logger=RunLogger(Path(tmp_path) / "gps-heading.csv"),
        clock=lambda: 10.0,
    )
    runtime.initialize(arm=True)
    manual.publish(autonomous_requested=True)

    acquiring = runtime.step()
    tracking = runtime.step()

    assert acquiring == MotionCommand(forward=0.1)
    assert tracking.forward > acquiring.forward
    assert tracking.yaw == 0.0
    runtime.shutdown()
