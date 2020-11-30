"""Execute a ROS node using the ezrassor_topic_switch module.

This node publishes messages from one of two input topics to an output topic.
It switches between inputs based on a mutable override status.

This file contains any ROS-specific objects required to execute the module as a
node--the rest of the module contains *no* references to ROS. This file, and
only this file, imports rclpy for ROS API access.

references:
  docs.python.org/3/library/__main__.html
  docs.ros2.org/foxy/api/rclpy/api.html
  github.com/ros2/examples/tree/dashing/rclpy
"""
import ezrassor_topic_switch as switch
import rclpy
import std_msgs.msg


NODE = "topic_switch"
MESSAGE_TYPE = "message_type"
OVERRIDE_TOPIC = "override"
PRIMARY_INPUT_TOPIC = "primary_input"
SECONDARY_INPUT_TOPIC = "secondary_input"
OUTPUT_TOPIC = "output"
QUEUE_SIZE = 10


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)
        node.declare_parameter(MESSAGE_TYPE)

        message_type = switch.load_message_type(
            node.get_parameter(MESSAGE_TYPE).value,
        )

        publish = node.create_publisher(
            message_type,
            OUTPUT_TOPIC,
            QUEUE_SIZE,
        ).publish

        override_status = switch.OverrideStatus()
        node.create_subscription(
            std_msgs.msg.Bool,
            OVERRIDE_TOPIC,
            lambda message: override_status.update(message.data),
            QUEUE_SIZE,
        )

        node.create_subscription(
            message_type,
            PRIMARY_INPUT_TOPIC,
            switch.conditionally_process(
                func=publish,
                override_status=override_status,
                expected_status=False,
            ),
            QUEUE_SIZE,
        )
        node.create_subscription(
            message_type,
            SECONDARY_INPUT_TOPIC,
            switch.conditionally_process(
                func=publish,
                override_status=override_status,
                expected_status=True,
            ),
            QUEUE_SIZE,
        )

        rclpy.spin(node)
    except switch.MessageLoadError as error:
        node.get_logger().error(str(error))
        exit(1)
    except KeyboardInterrupt:
        pass
