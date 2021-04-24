"""Launch a topic switch node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create a topic switch node with a launch description."""
    message_type_argument = DeclareLaunchArgument(
        "message_type",
        default_value="std_msgs.msg.String",
        description="topic message type",
    )
    channels_argument = DeclareLaunchArgument(
        "channels",
        default_value="2",
        description="number of desired channels",
    )

    topic_switch_node = Node(
        package="ezrassor_topic_switch",
        executable="topic_switch",
        parameters=[
            {
                "message_type": LaunchConfiguration("message_type"),
                "channels": LaunchConfiguration("channels"),
            },
        ],
        output={"both": "screen"},
    )

    return LaunchDescription(
        [
            message_type_argument,
            channels_argument,
            topic_switch_node,
        ]
    )
