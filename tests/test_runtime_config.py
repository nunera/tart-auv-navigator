import json

from auv.config import RuntimeConfig


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
