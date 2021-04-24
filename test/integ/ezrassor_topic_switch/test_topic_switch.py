"""Integration tests for the topic_switch using the launch_testing framework.

These tests ensure that the topic_switch behaves as expected. To run the tests,
build and source the package, then execute this command from the top-level
directory of the package:
  launch_test test/integ/ezrassor_topic_switch/test_topic_switch.py

references:
  en.wikipedia.org/wiki/Integration_testing
  github.com/ros2/launch/blob/master/launch_testing/README.md
"""
import ament_index_python.packages as ament
import launch
import launch.launch_description_sources as description_sources
import rclpy
import std_msgs.msg
import time
import unittest

NODE = "test_topic_switch"
PACKAGE = "ezrassor_topic_switch"
TOPIC_SWITCH_LAUNCH_FILE_FORMAT = "{0}/launch/topic_switch.py"
TEST_MESSAGE_TYPE = "std_msgs.msg.String"
TEST_CHANNELS = "2"
INPUT_TOPIC_FORMAT = "input{0}"
CHANNEL_TOPIC = "channel"
OUTPUT_TOPIC = "output"
QUEUE_SIZE = 10
CATCHUP_TIME = 1
TIMEOUT = 0.5


def generate_test_description(ready_fn):
    """Create a test description with the topic_switch launch file.

    This function is required by the launch_testing framework. It generates a
    new launch description that includes all of the nodes and additional launch
    files required for the tests. It also includes an "OpaqueFunction" that
    indicates to the framework that the tests should be launched immediately.

    This OpaqueFunction will be removed in ROS Foxy.
    """
    topic_switch_launch_file = TOPIC_SWITCH_LAUNCH_FILE_FORMAT.format(
        ament.get_package_share_directory(PACKAGE),
    )

    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                description_sources.PythonLaunchDescriptionSource(
                    topic_switch_launch_file,
                ),
                launch_arguments={
                    "message_type": TEST_MESSAGE_TYPE,
                    "channels": TEST_CHANNELS,
                }.items(),
            ),
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]
    )


class TopicSwitchIntegrationTests(unittest.TestCase):
    """A suite of integration tests for the topic_switch."""

    @classmethod
    def setUpClass(*arguments):
        """Initialize ROS before testing begins.

        This method name is required by unittest.
        """
        rclpy.init()

    def setUp(self):
        """Initialize testing infrastructure before each test.

        This method name is required by unittest.
        """
        self._node = rclpy.create_node(NODE)

        # Store output messages from the topic_switch in a list.
        self._received_messages = []
        self._node.create_subscription(
            std_msgs.msg.String,
            OUTPUT_TOPIC,
            lambda message: self._received_messages.append(message.data),
            QUEUE_SIZE,
        )

        # Create publishers for each input topic.
        self._input_topics = []
        for channel in range(int(TEST_CHANNELS)):
            self._input_topics.append(
                self._node.create_publisher(
                    std_msgs.msg.String,
                    INPUT_TOPIC_FORMAT.format(channel),
                    QUEUE_SIZE,
                )
            )

        # Create a publisher for the channel topic.
        self._channel_topic = self._node.create_publisher(
            std_msgs.msg.Int8,
            CHANNEL_TOPIC,
            QUEUE_SIZE,
        )

        # Sleep for some time to give ROS a moment to warm up.
        time.sleep(CATCHUP_TIME)

    def tearDown(self):
        """Destroy testing infrastructure after each test.

        This method name is required by unittest.
        """
        self._node.destroy_node()

    @classmethod
    def tearDownClass(*arguments):
        """Shut down ROS after testing is complete.

        This method name is required by unittest.
        """
        rclpy.shutdown()

    def _publish(self, raw_messages, channel):
        """Publish a series of messages to an input topic.

        After each message is published, spin the testing node once to process
        any output from the topic_switch.
        """
        for raw_message in raw_messages:
            message = std_msgs.msg.String()
            message.data = raw_message
            self._input_topics[channel].publish(message)
            rclpy.spin_once(self._node, timeout_sec=TIMEOUT)

    def _change_channel(self, channel):
        """Change the topic_switch's channel."""
        message = std_msgs.msg.Int8()
        message.data = channel
        self._channel_topic.publish(message)

        # Wait a moment to prevent dropped messages.
        time.sleep(CATCHUP_TIME)

    def _clear_received_messages(self):
        """Clear all received messages."""
        self._received_messages = []

    def _verify_received_messages(self, expected_messages):
        """Verify the received messages are as expected."""
        self.assertEqual(self._received_messages, expected_messages)

    def test_topic_switch_processes_messages_based_on_current_channel(self):
        """Should process messages appropriately based on the current channel.

        This test sets the topic_switch channel to 0, then sends messages
        through input0 and input1. Only input0 messages should be processed.

        Next, the test sets the topic_switch channel to 1, then sends messages
        through input0 and input1. Only input1 messages should be processed.

        Finally, the test sets the topic_switch channel back to 0 and sends
        messages through input0 and input1. Only input0 messages should be
        processed.
        """
        self._change_channel(0)
        self._clear_received_messages()
        self._publish(["expected1", "expected2", "expected3"], channel=0)
        self._publish(["unexpected1", "unexpected2", "unexpected3"], channel=1)
        self._verify_received_messages(["expected1", "expected2", "expected3"])

        self._change_channel(1)
        self._clear_received_messages()
        self._publish(["unexpected4", "unexpected5", "unexpected6"], channel=0)
        self._publish(["expected4", "expected5", "expected6"], channel=1)
        self._verify_received_messages(["expected4", "expected5", "expected6"])

        self._change_channel(0)
        self._clear_received_messages()
        self._publish(["expected7", "expected8", "expected9"], channel=0)
        self._publish(["unexpected7", "unexpected8", "unexpected9"], channel=1)
        self._verify_received_messages(["expected7", "expected8", "expected9"])
