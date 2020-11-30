"""Process items based on the override status."""
import threading


def conditionally_process(func, override_status, expected_status):
    """Create a function that processes items based on the override status."""

    def _conditionally_process(item):
        """Process an item if the override status equals an expected status."""
        if override_status == expected_status:
            return func(item)

    return _conditionally_process


class OverrideStatus:
    """Maintain a thread-safe override status.

    This class wraps a raw boolean and allows us to pass this boolean around
    using pass-by-reference. Asynchronous updates to the boolean are available
    to all objects and functions that hold a reference.
    """

    def __init__(self):
        """Initialize an override status with a thread lock."""
        self._status = False
        self._lock = threading.Lock()

    def __eq__(self, other_boolean):
        """Compare the override status with a boolean."""
        with self._lock:
            return self._status == other_boolean

    def update(self, new_status):
        """Update the override status."""
        with self._lock:
            self._status = new_status
