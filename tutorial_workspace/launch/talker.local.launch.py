from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Talker example that launches everything locally."""
    ld = LaunchDescription()

    listener_node = Node(
        package="fogros2_examples", 
        executable="listener", 
        output="screen"
    )
    talker_node = Node(
        package="fogros2_examples",
        executable="talker", 
        output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
