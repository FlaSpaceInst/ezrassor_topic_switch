"""Initialize the ezrassor_topic_switch module.

This file indicates that this directory is a Python module and also exposes
functions and classes for public use. Functions and classes not imported here
should be prefixed with a single underscore and not used outside of the module.

references:
  docs.python.org/3/tutorial/modules.html#packages
  www.python.org/dev/peps/pep-0008/#descriptive-naming-styles
"""
from .load import load_channels, load_message_type, LoadError
from .process import Multiplexer
