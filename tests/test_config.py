from auv.navigation import NavigationController


def test_empty_waypoint_list_is_safe_for_manual_only_operation():
    controller = NavigationController([])

    controller.enable()

    assert controller.complete
    assert not controller.enabled
