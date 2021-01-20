"""Test the processing machinery in this module."""
import ezrassor_topic_switch as switch


def test_multiplexer_with_items_in_current_channel():
    """Should process items on current channel."""
    multiplexer = switch.Multiplexer(process_output=lambda message: message)
    multiplexer.change_channel(0)

    assert multiplexer.offer_input(0, "msg1") == "msg1"
    assert multiplexer.offer_input(0, "msg2") == "msg2"
    assert multiplexer.offer_input(0, "msg3") == "msg3"


def test_multiplexer_ignores_items_in_other_channels():
    """Should not process items in other channels."""
    multiplexer = switch.Multiplexer(process_output=lambda message: message)
    multiplexer.change_channel(0)

    assert multiplexer.offer_input(1, "msg1") is None
    assert multiplexer.offer_input(1, "msg2") is None
    assert multiplexer.offer_input(1, "msg3") is None


def test_multiplexer_changes_channels_correctly():
    """Should process items in whichever channel is current."""
    multiplexer = switch.Multiplexer(process_output=lambda message: message)

    multiplexer.change_channel(0)
    assert multiplexer.offer_input(0, "msg1") == "msg1"
    assert multiplexer.offer_input(0, "msg2") == "msg2"
    assert multiplexer.offer_input(1, "msg3") is None

    multiplexer.change_channel(1)
    assert multiplexer.offer_input(0, "msg1") is None
    assert multiplexer.offer_input(0, "msg2") is None
    assert multiplexer.offer_input(1, "msg3") == "msg3"
