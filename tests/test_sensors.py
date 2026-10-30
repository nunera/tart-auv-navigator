from auv.models import PositionSample
from auv.sensors import GpsTrackHeading, NmeaGpsReader, tds_ppm


class Lines:
    def __init__(self, *lines):
        self.lines = iter(lines)

    def readline(self):
        return next(self.lines, b"")


def nmea(payload):
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    return f"${payload}*{checksum:02X}\r\n".encode()


def test_void_fix_clears_last_gps_position():
    valid = nmea("GPRMC,123519,A,4807.038,N,01131.000,E,0.0,0.0,230394,,")
    void = nmea("GPRMC,123520,V,,,,,,,230394,,")
    reader = NmeaGpsReader(Lines(valid, void))

    assert reader.read(now=1.0).valid
    no_fix = reader.read(now=1.1)

    assert not no_fix.valid
    assert no_fix.latitude is None
    assert no_fix.longitude is None


def test_bad_checksum_never_fabricates_position():
    reader = NmeaGpsReader(Lines(b"$GPRMC,123519,A,4807.038,N,01131.000,E*00\r\n"))

    sample = reader.read(now=1.0)

    assert not sample.valid
    assert sample.latitude is None


def test_fixes_before_a_gap_are_not_averaged_into_the_next_fix():
    before = nmea("GPRMC,123519,A,4807.038,N,01131.000,E,0.0,0.0,230394,,")
    void = nmea("GPRMC,123520,V,,,,,,,230394,,")
    after = nmea("GPRMC,123521,A,4808.038,N,01131.000,E,0.0,0.0,230394,,")
    reader = NmeaGpsReader(Lines(before, void, after))
    reader.read(now=1.0)
    reader.read(now=2.0)

    sample = reader.read(now=3.0)

    assert abs(sample.latitude - (48 + 8.038 / 60)) < 1e-9


def test_gps_track_heading_needs_real_displacement_and_holds_between_fixes():
    tracker = GpsTrackHeading(min_distance_m=2.0)
    start = PositionSample(29.3136, -94.8170, 0.0, True)
    jitter = PositionSample(29.31361, -94.8170, 1.0, True)  # ~1 m: GPS noise, not travel
    moved = PositionSample(29.31363, -94.8170, 2.0, True)  # ~3 m north

    assert not tracker.update(start).valid
    assert not tracker.update(jitter).valid
    heading = tracker.update(moved)
    assert heading.valid and (heading.degrees < 1.0 or heading.degrees > 359.0)
    assert tracker.update(moved) == heading  # repeated fix keeps the heading

    assert not tracker.update(PositionSample(None, None, 3.0, False)).valid


def test_tds_formula_uses_voltage_squared_middle_term():
    assert tds_ppm(2.0, 25.0) == 879.35
