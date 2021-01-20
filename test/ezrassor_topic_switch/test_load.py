"""Test the loading machinery in this module."""
import ezrassor_topic_switch as switch
import numbers


def test_load_channels_with_none():
    """Should fail to load channels if channels is None."""
    try:
        switch.load_channels(None)
        assert False, "load_channels() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "channels argument is not set" in error.message


def test_load_channels_with_alphabetic_characters():
    """Should fail to load channels if alphabetic characters are present."""
    try:
        switch.load_channels("abc")
        assert False, "load_channels() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "channels argument must be a number" in error.message


def test_load_channels_with_negative_number():
    """Should fail to load channels if number is negative."""
    try:
        switch.load_channels("-1")
        assert False, "load_channels() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "must have more than 1 channel" in error.message


def test_load_channels_with_number_above_upper_bound():
    """Should fail to load channels if number is above the upper bound."""
    try:
        switch.load_channels("200", upper_bound=99)
        assert False, "load_channels() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "must have fewer than 100 channels" in error.message


def test_load_channels_with_valid_number():
    """Should load channels properly."""
    channels = switch.load_channels("4")
    assert channels == 4


def test_load_message_type_with_none():
    """Should fail to load type if message type is None."""
    try:
        switch.load_message_type(None)
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "message type is not set" in error.message


def test_load_message_type_with_nonstring_message_type():
    """Should fail to load type if message type is not a string."""
    try:
        switch.load_message_type(1)
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "message type must be a string" in error.message


def test_load_message_type_from_malformed_message_type():
    """Should fail to load type if message type is misspelled/malformed."""
    try:
        switch.load_message_type("malformed")
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "must include periods" in error.message


def test_load_message_type_from_nonexistent_module():
    """Should fail to load type if the Python module doesn't exist."""
    try:
        switch.load_message_type("nonexistent.module.Class")
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "no module named 'nonexistent.module'" in error.message


def test_load_message_type_with_nonexistent_message_type():
    """Should fail to load type if the Python class doesn't exist."""
    try:
        switch.load_message_type("numbers.NonexistentClass")
        assert False, "load_message_type() should have failed"
    except Exception as error:
        assert isinstance(error, switch.LoadError)
        assert "no message named 'NonexistentClass'" in error.message


def test_load_message_type_from_valid_message_type():
    """Should load type and return it when given a valid message type."""
    message_type = switch.load_message_type("numbers.Complex")
    assert message_type is numbers.Complex
