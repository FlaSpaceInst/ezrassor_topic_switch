"""Test the processing machinery in this module."""
import ezrassor_topic_switch as switch


def test_conditionally_process_when_status_matches_expected():
    """Should process items since statuses match."""
    override_status = switch.OverrideStatus()
    override_status.update(False)
    conditionally_process = switch.conditionally_process(
        lambda number: number,
        override_status,
        False,
    )

    assert conditionally_process(1) == 1
    assert conditionally_process(2) == 2
    assert conditionally_process(3) == 3


def test_conditionally_process_when_status_does_not_match_expected():
    """Should not process items since statuses do not match."""
    override_status = switch.OverrideStatus()
    override_status.update(True)
    conditionally_process = switch.conditionally_process(
        lambda number: number,
        override_status,
        False,
    )

    assert conditionally_process(1) is None
    assert conditionally_process(2) is None
    assert conditionally_process(3) is None


def test_conditionally_process_when_status_is_updated():
    """Should process items at first and then stop after status changes."""
    override_status = switch.OverrideStatus()
    override_status.update(False)
    conditionally_process = switch.conditionally_process(
        lambda number: number,
        override_status,
        False,
    )

    assert conditionally_process(1) == 1
    assert conditionally_process(2) == 2

    override_status.update(True)

    assert conditionally_process(3) is None
