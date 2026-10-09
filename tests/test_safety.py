from auv.models import HealthSnapshot, ManualInput, MotionCommand
from auv.safety import ControlMode, SafetyController


def healthy(*, heading_fresh=True):
    return HealthSnapshot(
        navigator_ready=True,
        input_fresh=True,
        gps_fresh=True,
        heading_fresh=heading_fresh,
    )


def test_amg_autonomy_requires_fresh_heading():
    decision = SafetyController().decide(
        mode=ControlMode.AUTONOMOUS,
        manual=ManualInput(),
        autonomous=MotionCommand(forward=0.2),
        health=healthy(heading_fresh=False),
    )

    assert not decision.allow_motion
    assert decision.command.is_neutral


def test_gps_heading_acquisition_is_explicitly_limited_by_navigation_command():
    decision = SafetyController().decide(
        mode=ControlMode.AUTONOMOUS,
        manual=ManualInput(),
        autonomous=MotionCommand(forward=0.1),
        health=healthy(heading_fresh=False),
        permit_gps_heading_acquisition=True,
    )

    assert decision.allow_motion
    assert decision.command == MotionCommand(forward=0.1)
