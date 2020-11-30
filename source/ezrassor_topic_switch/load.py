"""Load a message type by name.

references:
  docs.python.org/3/library/importlib.html
"""
import importlib


def load_message_type(type_name):
    """Load a message type from its corresponding Python module."""

    if type_name is None:
        raise MessageLoadError("message type is not set")

    if not isinstance(type_name, str):
        raise MessageLoadError("message type must be a string")

    try:
        module_name, message_name = type_name.rsplit(".", 1)
    except ValueError:
        raise MessageLoadError(
            "message type must include periods, like 'std_msgs.msg.String'",
        )

    try:
        module = importlib.import_module(module_name)
    except ImportError:
        raise MessageLoadError(f"no module named '{module_name}'")

    try:
        return vars(module)[message_name]
    except KeyError:
        raise MessageLoadError(
            f"no message named '{message_name}' in module '{module_name}'"
        )


class MessageLoadError(Exception):
    """Encapsulate message loading errors."""

    def __init__(self, message):
        """Initialize this error with a message."""
        self.message = message

    def __str__(self):
        """Create a human-readable representation of this error."""
        return self.message
