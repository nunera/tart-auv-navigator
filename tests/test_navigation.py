from auv.models import HeadingSample, NavigationObservation, PositionSample
from auv.navigation import AutonomyLimits, NavigationController, Waypoint


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


def test_run_limits_are_off_when_not_configured():
    controller = NavigationController([TARGET])
    controller.enable()

    assert controller.tick(observation(heading=0, now=0.0)).forward > 0
    assert controller.tick(observation(heading=0, now=100_000.0)).forward > 0
    assert controller.abort_reason is None


def test_waypoint_timeout_stops_run_and_survives_disable():
    controller = NavigationController([TARGET], limits=AutonomyLimits(max_waypoint_s=30))
    controller.enable()
    controller.tick(observation(heading=0, now=0.0))

    assert controller.tick(observation(heading=0, now=31.0)).is_neutral
    assert "waypoint 1" in controller.abort_reason
    controller.disable()
    assert controller.abort_reason is not None

    controller.enable()
    assert controller.abort_reason is None
    assert controller.tick(observation(heading=0, now=40.0)).forward > 0


def test_distance_limit_measures_from_where_autonomy_started():
    controller = NavigationController([Waypoint(29.4, -94.8170)], limits=AutonomyLimits(max_distance_m=50))
    controller.enable()
    controller.tick(observation(heading=0, now=0.0))

    near = PositionSample(latitude=29.3140, longitude=-94.8170, observed_at=1.0, valid=True)  # ~44 m north
    far = PositionSample(latitude=29.3141, longitude=-94.8170, observed_at=2.0, valid=True)  # ~56 m north
    assert controller.tick(observation(heading=0, position=near, now=1.0)).forward > 0
    assert controller.tick(observation(heading=0, position=far, now=2.0)).is_neutral
    assert "exceeds 50 m" in controller.abort_reason
