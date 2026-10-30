from pathlib import Path

from auv.actuation import ActuatorState, NavigatorActuator
from auv.input import SharedManualInput
from auv.logging import RunLogger
from auv.models import HeadingSample, MotionCommand, PositionSample
from auv.navigation import AutonomyLimits, NavigationController, Waypoint
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
    runtime.step()  # controls start released
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


def test_manual_control_waits_for_release_at_startup_and_after_autonomy(tmp_path):
    runtime, manual, _ = make_runtime(tmp_path)
    manual.publish(forward=0.5)
    assert runtime.step().is_neutral
    assert runtime.status.reason == "release controls to resume manual"

    manual.publish(forward=0.0)
    runtime.step()
    manual.publish(forward=0.5)
    assert runtime.step().forward == 0.5

    manual.publish(autonomous_requested=True)
    runtime.step()
    manual.publish(autonomous_requested=False)
    assert runtime.step().is_neutral, "stale manual command resumed after leaving autonomy"

    manual.publish(forward=0.0)
    runtime.step()
    manual.publish(forward=0.3)
    assert runtime.step().forward == 0.3
    runtime.shutdown()


class StepClock:
    def __init__(self, now=10.0):
        self.now = now

    def __call__(self):
        return self.now


def test_aborted_autonomous_run_stays_stopped_until_operator_toggles_autonomy(tmp_path):
    clock = StepClock()
    runtime = ControlRuntime(
        actuator=NavigatorActuator(FakePwm(), sleep_fn=lambda _: None),
        manual_input=(manual := SharedManualInput()),
        position_reader=FixedPosition(),
        heading_reader=FixedHeading(),
        navigation=NavigationController([Waypoint(29.3146, -94.8170)], limits=AutonomyLimits(max_duration_s=5)),
        safety=SafetyController(),
        logger=RunLogger(Path(tmp_path) / "limits.csv"),
        clock=clock,
    )
    runtime.initialize(arm=True)
    manual.publish(autonomous_requested=True)

    assert runtime.step().forward > 0
    clock.now += 6
    assert runtime.step().is_neutral
    assert "exceeded 5 s" in runtime.status.reason
    clock.now += 1
    assert runtime.step().is_neutral
    assert runtime.actuator.state is ActuatorState.ARMED, "limit breach must leave manual control available"

    manual.publish(autonomous_requested=False)
    runtime.step()
    manual.publish(autonomous_requested=True)
    assert runtime.step().forward > 0
    runtime.shutdown()


class OneHertzGps:
    """Returns each fix for ten 0.1 s ticks, like a 1 Hz receiver under a 10 Hz loop."""

    def __init__(self, fixes):
        self.fixes = fixes
        self.last = None

    def read(self, now):
        index = round((now - 10.0) * 10) // 10
        if self.last is None or self.last[0] != index:
            latitude, longitude = self.fixes[min(index, len(self.fixes) - 1)]
            self.last = (index, PositionSample(latitude, longitude, now, True))
        return self.last[1]


def test_gps_track_heading_keeps_steering_between_fixes(tmp_path):
    clock = StepClock()
    # Heading north-east at roughly 3 m per fix toward a waypoint due north.
    fixes = [(29.3136 + 0.00002 * index, -94.8170 + 0.00002 * index) for index in range(5)]
    runtime = ControlRuntime(
        actuator=NavigatorActuator(FakePwm(), sleep_fn=lambda _: None),
        manual_input=(manual := SharedManualInput()),
        position_reader=OneHertzGps(fixes),
        heading_reader=FixedHeading(),
        navigation=NavigationController([Waypoint(29.3156, -94.8170)], heading_source="gps"),
        safety=SafetyController(),
        logger=RunLogger(Path(tmp_path) / "gps-1hz.csv"),
        clock=clock,
    )
    runtime.initialize(arm=True)
    manual.publish(autonomous_requested=True)

    commands = []
    for _ in range(40):
        commands.append(runtime.step())
        clock.now += 0.1

    steering = commands[10:]
    assert all(command.yaw < 0 for command in steering), "track heading dropped out between GPS fixes"
    runtime.shutdown()


def test_amg_autonomy_keeps_driving_between_one_hertz_fixes(tmp_path):
    clock = StepClock()
    runtime = ControlRuntime(
        actuator=NavigatorActuator(FakePwm(), sleep_fn=lambda _: None),
        manual_input=(manual := SharedManualInput()),
        position_reader=OneHertzGps([(29.3136, -94.8170)]),
        heading_reader=FixedHeading(),
        navigation=NavigationController([Waypoint(29.3146, -94.8170)]),
        safety=SafetyController(),
        logger=RunLogger(Path(tmp_path) / "amg-1hz.csv"),
        clock=clock,
    )
    runtime.initialize(arm=True)
    manual.publish(autonomous_requested=True)

    for _ in range(10):
        assert runtime.step().forward > 0, "thrust pulsed off while waiting for the next fix"
        clock.now += 0.1
    runtime.shutdown()
