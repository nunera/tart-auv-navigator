from auv.models import HeadingSample, NavigationObservation, PositionSample
from auv.navigation import NavigationController, Waypoint


START = PositionSample(latitude=29.3136, longitude=-94.8170, observed_at=0.0, valid=True)
TARGET = Waypoint(latitude=29.3146, longitude=-94.8170)

def observation(*, heading=None, position=START, now=1.0):
    current_position = PositionSample(
        latitude=position.latitude,
        longitude=position.longitude,
        observed_at=now,
        valid=position.valid,
        error=position.error,
    )
    return NavigationObservation(
        position=current_position,
        heading=HeadingSample(degrees=heading, observed_at=now, valid=heading is not None),
        now=now,
    )
def test_missing_amg_heading_stops_autonomy():
    controller = NavigationController([TARGET], heading_source="amg")
    controller.enable()

    command = controller.tick(observation(heading=None))

    assert command.forward == 0
    assert command.yaw == 0
    assert command.vertical == 0


def test_missing_gps_heading_creeps_straight_without_turning():
    controller = NavigationController([TARGET], heading_source="gps", max_heading_acquisition_forward=0.1)
    controller.enable()

    command = controller.tick(observation(heading=None))

    assert 0 < command.forward <= 0.1
    assert command.yaw == 0


def test_invalid_position_stops_autonomy():
    controller = NavigationController([TARGET])
    controller.enable()
    invalid = PositionSample(latitude=None, longitude=None, observed_at=1.0, valid=False)

    assert controller.tick(observation(heading=0, position=invalid)).is_neutral


def test_completion_stays_neutral_after_position_drifts_away():
    controller = NavigationController([Waypoint(START.latitude, START.longitude)], arrival_radius_m=5)
    controller.enable()
    assert controller.tick(observation(heading=0)).is_neutral
    assert controller.complete

    drifted = PositionSample(latitude=29.315, longitude=-94.817, observed_at=2, valid=True)
    assert controller.tick(observation(heading=0, position=drifted, now=2)).is_neutral


def test_heading_error_wraps_across_north():
    controller = NavigationController([TARGET], heading_source="amg", turn_gain=0.02)
    controller.enable()
    command = controller.tick(observation(heading=350))

    assert command.yaw > 0
    assert abs(command.yaw) < 0.5
