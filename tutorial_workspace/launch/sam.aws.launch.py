from launch_ros.actions import Node

import fogros2



def generate_launch_description():
    """Talker example that launches the listener on AWS."""
    ld = fogros2.FogROSLaunchDescription()
    machine1 = fogros2.AWSCloudInstance(
        region="us-west-1", ec2_instance_type="g4dn.4xlarge", ami_image="ami-05394da34368a5cc1",
        disk_size=35
    )

    sam_client_node = Node(
        package="fogros2_tutorial", executable="sam_client", output="screen"
    )

    sam_server_node = fogros2.CloudNode(
        package="fogros2_tutorial",
        executable="sam_server",
        output="screen",
        machine=machine1,
    )
    ld.add_action(sam_client_node)
    ld.add_action(sam_server_node)
    return ld
