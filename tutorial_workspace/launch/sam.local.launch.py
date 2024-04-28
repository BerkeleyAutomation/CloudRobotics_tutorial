from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Sam pubsub example that launches everything locally."""
    ld = LaunchDescription()

    listener_node = Node(
        package="sam_ros", executable="sam_sub", output="screen"
    )
    talker_node = Node(
        package="sam_ros", executable="sam_pub", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld