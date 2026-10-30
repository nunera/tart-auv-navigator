import json

from auv.config import RuntimeConfig
from auv.navigation import AutonomyLimits


def test_runtime_configuration_is_explicit_and_does_not_require_waypoints(tmp_path):
    path = tmp_path / "vehicle.json"
    path.write_text(
        json.dumps(
            {
                "channels": [3, 4, 5],
                "input": "keyboard",
                "speed_scale": 0.4,
                "waypoints": [],
            }
        )
    )

    config = RuntimeConfig.load(path)

    assert config.channels == (3, 4, 5)
    assert config.speed_scale == 0.4
    assert config.waypoints == []
    assert config.autonomy_limits == AutonomyLimits()


def test_autonomy_limits_are_individually_optional_and_reject_typos(tmp_path):
    path = tmp_path / "vehicle.json"
    path.write_text(json.dumps({"autonomy_limits": {"max_duration_s": 300, "max_distance_m": None}}))

    limits = RuntimeConfig.load(path).autonomy_limits
    assert limits == AutonomyLimits(max_duration_s=300.0)

    path.write_text(json.dumps({"autonomy_limits": {"max_duration": 300}}))
    try:
        RuntimeConfig.load(path)
    except ValueError as error:
        assert "max_duration" in str(error)
    else:
        raise AssertionError("misspelled limit was silently ignored")
