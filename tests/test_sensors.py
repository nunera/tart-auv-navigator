from auv.sensors import NmeaGpsReader, tds_ppm


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


def test_tds_formula_uses_voltage_squared_middle_term():
    assert tds_ppm(2.0, 25.0) == 879.35
