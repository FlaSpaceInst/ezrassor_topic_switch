"""Process data with a channeled multiplexer.

references:
  en.wikipedia.org/wiki/Multiplexer
"""
import threading


class Multiplexer:
    """Process and output data from a selected input channel."""

    def __init__(self, process_output):
        """Initialize this multiplexer."""
        self._process_output = process_output
        self._current_channel = 0
        self._lock = threading.Lock()

    def change_channel(self, new_channel):
        """Change the current multiplexer channel."""
        with self._lock:
            self._current_channel = new_channel

    def offer_input(self, channel, data):
        """Process and output data if the given channel is currently active."""
        with self._lock:
            if channel == self._current_channel:
                return self._process_output(data)
