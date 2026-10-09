from auv.actuation import ActuatorState, NavigatorActuator
from auv.models import MotionCommand


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


def duties_for(backend, channel):
    return [call[2] for call in backend.calls if call[:2] == ("duty", channel)]


def test_arm_command_and_disarm_neutralizes_before_disabling_pwm():
    backend = FakePwm()
    actuator = NavigatorActuator(backend, channels=(0, 1, 2), frequency_hz=50, sleep_fn=lambda _: None)

    actuator.initialize()
    actuator.arm()
    actuator.command(MotionCommand(forward=1, yaw=0, vertical=0.25))
    actuator.disarm("test")

    assert actuator.state is ActuatorState.DISARMED
    assert duties_for(backend, 0)[-1] == 0.075
    assert duties_for(backend, 1)[-1] == 0.075
    assert duties_for(backend, 2)[-1] == 0.075
    assert backend.calls[-1] == ("enabled", False)


def test_neutral_command_ignores_horizontal_trim():
    backend = FakePwm()
    actuator = NavigatorActuator(backend, channels=(0, 1, 2), horizontal_trim=0.25, sleep_fn=lambda _: None)
    actuator.initialize()
    actuator.arm()

    actuator.command(MotionCommand())

    assert duties_for(backend, 0)[-1] == 0.075
    assert duties_for(backend, 1)[-1] == 0.075
    assert duties_for(backend, 2)[-1] == 0.075


def test_fault_latches_and_requires_explicit_rearm():
    backend = FakePwm()
    actuator = NavigatorActuator(backend, channels=(0, 1, 2), sleep_fn=lambda _: None)
    actuator.initialize()
    actuator.arm()
    actuator.fault("lost heading")

    assert actuator.state is ActuatorState.FAULT
    assert backend.calls[-1] == ("enabled", False)

    try:
        actuator.command(MotionCommand(forward=1))
    except RuntimeError as error:
        assert "fault" in str(error).lower()
    else:
        raise AssertionError("faulted actuator accepted a motor command")
