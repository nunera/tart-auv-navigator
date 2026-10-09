from auv.calibration import MagnetometerCalibration, fit_axis_aligned_magnetometer


def test_axis_aligned_fit_removes_offset_and_equalizes_axes(tmp_path):
    samples = [
        (12.0, 0.0, 0.0), (-8.0, 0.0, 0.0),
        (2.0, 5.0, 0.0), (2.0, -5.0, 0.0),
        (2.0, 0.0, 20.0), (2.0, 0.0, -20.0),
    ] * 4

    calibration = fit_axis_aligned_magnetometer(samples)
    assert calibration.offset == (2.0, 0.0, 0.0)
    assert calibration.apply((12.0, 0.0, 0.0))[0] == calibration.apply((2.0, 5.0, 0.0))[1]
    assert calibration.apply((2.0, 5.0, 0.0))[1] == calibration.apply((2.0, 0.0, 20.0))[2]
    path = tmp_path / "mag.json"
    calibration.save(path)
    assert MagnetometerCalibration.load(path) == calibration
