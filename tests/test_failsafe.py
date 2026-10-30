import os
import signal

from auv.failsafe import CLEAN_EXIT, HEARTBEAT, force_neutral, wait_for_trip


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


def pipe_with(data, *, close_writer):
    read_fd, write_fd = os.pipe()
    os.write(write_fd, data)
    if close_writer:
        os.close(write_fd)
        write_fd = None
    return read_fd, write_fd


def test_clean_exit_after_heartbeats_does_not_trip():
    read_fd, write_fd = pipe_with(HEARTBEAT * 3 + CLEAN_EXIT, close_writer=True)

    assert wait_for_trip(read_fd, timeout_s=0.5) is None
    os.close(read_fd)


def test_pipe_closed_without_clean_exit_trips():
    read_fd, _ = pipe_with(HEARTBEAT * 3, close_writer=True)

    assert "exited" in wait_for_trip(read_fd, timeout_s=0.5)
    os.close(read_fd)


def test_missing_heartbeat_trips_while_control_process_is_alive():
    read_fd, write_fd = pipe_with(HEARTBEAT, close_writer=False)

    assert "no heartbeat" in wait_for_trip(read_fd, timeout_s=0.05)
    os.close(read_fd)
    os.close(write_fd)


def test_force_neutral_kills_hung_parent_then_writes_neutral_and_disables_pwm():
    backend, kills = FakePwm(), []
    parents = iter([1234, 1234, 1])  # alive, still alive after kill, then reparented

    force_neutral(
        "no heartbeat",
        parent_pid=1234,
        backend=backend,
        channels=(0, 1, 2),
        frequency_hz=50,
        getppid=lambda: next(parents),
        kill=lambda pid, signum: kills.append((pid, signum)),
        sleep=lambda _: None,
    )

    assert kills == [(1234, signal.SIGKILL)]
    duties = {call[1]: call[2] for call in backend.calls if call[0] == "duty"}
    assert duties == {0: 0.075, 1: 0.075, 2: 0.075}
    assert backend.calls[-1] == ("enabled", False)


def test_force_neutral_never_signals_a_reused_pid_after_parent_died():
    kills = []

    force_neutral(
        "exited",
        parent_pid=1234,
        backend=FakePwm(),
        channels=(0, 1, 2),
        frequency_hz=50,
        getppid=lambda: 1,
        kill=lambda pid, signum: kills.append(pid),
        sleep=lambda _: None,
    )

    assert kills == []
