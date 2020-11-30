"""Execute a ROS node using the ezrassor_topic_switch module.

This node publishes messages from a number of input topics to an output topic.
It switches between inputs based on a specified channel number.

This file contains any ROS-specific objects required to execute the module as a
node--the rest of the module contains *no* references to ROS. This file, and
only this file, imports rclpy for ROS API access.

references:
  docs.python.org/3/library/__main__.html
  docs.ros2.org/foxy/api/rclpy/api.html
  github.com/ros2/examples/tree/dashing/rclpy
"""
import ezrassor_topic_switch as switch
import functools
import rclpy
import std_msgs.msg


NODE = "topic_switch"
MESSAGE_TYPE = "message_type"
CHANNELS = "channels"
CHANNEL_TOPIC = "channel"
INPUT_TOPIC_FORMAT = "input{0}"
OUTPUT_TOPIC = "output"
QUEUE_SIZE = 10


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Declare and process node parameters.
        node.declare_parameter(MESSAGE_TYPE)
        node.declare_parameter(CHANNELS)
        message_type = switch.load_message_type(
            node.get_parameter(MESSAGE_TYPE).value,
        )
        channels = switch.load_channels(node.get_parameter(CHANNELS).value)

        # Create a multiplexer.
        publish = node.create_publisher(
            message_type,
            OUTPUT_TOPIC,
            QUEUE_SIZE,
        ).publish
        multiplexer = switch.Multiplexer(process_output=publish)
        node.create_subscription(
            std_msgs.msg.Int8,
            CHANNEL_TOPIC,
            lambda message: multiplexer.change_channel(message.data),
            QUEUE_SIZE,
        )

        # Create a subscriber for each channel. Channel numbers are pre-loaded
        # into each subscriber's callback with functools.partial().
        for channel in range(channels):
            node.create_subscription(
                message_type,
                INPUT_TOPIC_FORMAT.format(channel),
                functools.partial(multiplexer.offer_input, channel),
                QUEUE_SIZE,
            )

        # Spin!
        rclpy.spin(node)
    except switch.LoadError as error:
        node.get_logger().error(str(error))
        exit(1)
    except KeyboardInterrupt:
        pass
