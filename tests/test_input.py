import threading

from auv.input import KeyboardInputWorker, SharedManualInput


def keyboard():
    manual = SharedManualInput()
    return KeyboardInputWorker(manual, threading.Event(), level=2), manual


def test_held_key_drives_until_auto_repeat_stops():
    worker, manual = keyboard()

    worker.handle_key("w", now=0.0)
    worker.tick(0.5)  # still inside the terminal's initial auto-repeat delay
    assert manual.snapshot().command.forward == 0.4

    for repeat_at in (0.55, 0.6, 0.65):
        worker.handle_key("w", now=repeat_at)
        worker.tick(repeat_at)
        assert manual.snapshot().command.forward == 0.4

    worker.tick(0.65 + worker.repeat_hold_s + 0.01)  # key released
    assert manual.snapshot().command.is_neutral


def test_single_tap_stops_after_first_press_hold():
    worker, manual = keyboard()

    worker.handle_key("d", now=0.0)
    worker.tick(worker.first_press_hold_s - 0.01)
    assert manual.snapshot().command.yaw == 0.4

    worker.tick(worker.first_press_hold_s + 0.01)
    assert manual.snapshot().command.is_neutral


def test_new_key_replaces_previous_motion_instead_of_accumulating():
    worker, manual = keyboard()

    worker.handle_key("w", now=0.0)
    worker.handle_key("s", now=0.1)
    worker.tick(0.1)

    assert manual.snapshot().command.forward == -0.4


def test_speed_level_scales_held_motion():
    worker, manual = keyboard()

    worker.handle_key("5", now=0.0)
    worker.handle_key("e", now=0.0)
    worker.tick(0.0)

    command = manual.snapshot().command
    assert (command.forward, command.yaw) == (1.0, 1.0)


def test_arrow_key_escape_sequence_is_not_read_as_letters():
    worker, manual = keyboard()

    worker.handle_bytes(b"\x1b[A", now=0.0)  # up arrow ends in 'A'
    worker.tick(0.0)

    assert manual.snapshot().command.is_neutral


def test_mode_toggle_and_estop_release_held_motion():
    worker, manual = keyboard()

    worker.handle_key("w", now=0.0)
    worker.handle_key("m", now=0.1)
    worker.tick(0.1)
    assert manual.snapshot().autonomous_requested
    assert manual.snapshot().command.is_neutral

    worker.handle_key("w", now=0.2)
    worker.handle_key(" ", now=0.3)
    worker.tick(0.3)
    assert manual.snapshot().emergency_stop_requested
    assert manual.snapshot().command.is_neutral
