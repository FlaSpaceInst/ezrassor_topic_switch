"""Load and transform objects.

references:
  docs.python.org/3/library/importlib.html
"""
import importlib


def load_channels(channels, upper_bound=99):
    """Load channel count from a string."""
    if channels is None:
        raise LoadError("channels argument is not set")

    try:
        converted_channels = int(channels)
    except ValueError:
        raise LoadError("channels argument must be a number")

    if converted_channels < 2:
        raise LoadError("must have more than 1 channel")

    if converted_channels > upper_bound:
        raise LoadError(
            f"must have fewer than {upper_bound + 1} channels (arbitrary)",
        )

    return converted_channels


def load_message_type(message_type):
    """Load a message type from its corresponding Python module."""

    if message_type is None:
        raise LoadError("message type is not set")

    if not isinstance(message_type, str):
        raise LoadError("message type must be a string")

    try:
        module_name, message_name = message_type.rsplit(".", 1)
    except ValueError:
        raise LoadError(
            "message type must include periods, like 'std_msgs.msg.String'",
        )

    try:
        module = importlib.import_module(module_name)
    except ImportError:
        raise LoadError(f"no module named '{module_name}'")

    try:
        return vars(module)[message_name]
    except KeyError:
        raise LoadError(
            f"no message named '{message_name}' in module '{module_name}'"
        )


class LoadError(Exception):
    """Encapsulate argument errors."""

    def __init__(self, message):
        """Initialize this error with a message."""
        self.message = message

    def __str__(self):
        """Create a human-readable representation of this error."""
        return self.message
