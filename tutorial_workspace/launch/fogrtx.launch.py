from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Sam pubsub example that launches everything locally."""
    ld = LaunchDescription()

    listener_node = Node(
        package="fogros2_tutorial", executable="image_publisher", output="screen"
    )
    talker_node = Node(
        package="fogros2_tutorial", executable="fog_rtx_recorder", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld