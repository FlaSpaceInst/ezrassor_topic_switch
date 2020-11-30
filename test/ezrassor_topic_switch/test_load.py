"""Test the loading machinery in this module."""
import ezrassor_topic_switch as switch
import std_msgs.msg


def test_load_message_type_without_type_name():
    """Should fail to load type if type name is None."""
    try:
        switch.load_message_type(None)
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.MessageLoadError)
        assert "message type is not set" in error.message


def test_load_message_type_with_nonstring_type_name():
    """Should fail to load type if type name is not a string."""
    try:
        switch.load_message_type(1)
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.MessageLoadError)
        assert "message type must be a string" in error.message


def test_load_message_type_from_malformed_type_name():
    """Should fail to load type if type name is misspelled/malformed."""
    try:
        switch.load_message_type("malformed")
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.MessageLoadError)
        assert "must include periods" in error.message


def test_load_message_type_from_nonexistent_module():
    """Should fail to load type if the Python module doesn't exist."""
    try:
        switch.load_message_type("nonstd_msgs.msg.String")
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.MessageLoadError)
        assert "no module named 'nonstd_msgs.msg'" in error.message


def test_load_message_type_with_nonexistent_message_type():
    """Should fail to load type if the Python class doesn't exist."""
    try:
        switch.load_message_type("std_msgs.msg.UltraString")
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.MessageLoadError)
        assert "no message named 'UltraString'" in error.message


def test_load_message_type_with_type_name():
    """Should load type and return it when given a valid type name."""
    message_type = switch.load_message_type("std_msgs.msg.String")
    assert message_type is std_msgs.msg.String
