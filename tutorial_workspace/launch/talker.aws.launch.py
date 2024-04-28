from launch_ros.actions import Node

import fogros2

def generate_launch_description():
    """Talker example that launches the listener on AWS."""
    ld = fogros2.FogROSLaunchDescription()
    machine1 = fogros2.AWSCloudInstance(
        region="us-west-1", ec2_instance_type="t2.xlarge", ami_image="ami-009d3d1fc74506b49"
    )

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )

    talker_node = fogros2.CloudNode(
        package="fogros2_examples",
        executable="talker",
        output="screen",
        machine=machine1,
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
