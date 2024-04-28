from launch_ros.actions import Node

import fogros2


def ami_image():
    # An AMI is an Amazon Web Services virtual machine image with a
    # pre-installed OS and dependencies.  We match the AMI in the
    # cloud to have the same OS release as the robot.  Currently we
    # support Ubuntu 20.04 and 22.04.

    import lsb_release

    ubuntu_release = lsb_release.get_os_release()["RELEASE"]

    if ubuntu_release == "20.04":
        return "ami-00f25057ddc9b310b"
    if ubuntu_release == "22.04":
        return "ami-0ade21c9f40e674b2" #intermediate image

    raise ValueError(f"No AMI for {ubuntu_release}")


def generate_launch_description():
    """Talker example that launches the listener on AWS."""
    ld = fogros2.FogROSLaunchDescription()
    machine1 = fogros2.AWSCloudInstance(
        region="us-west-1", ec2_instance_type="g4dn.xlarge", ami_image=ami_image(),
        disk_size=35
    )

    listener_node = Node(
        package="sam_ros", executable="sam_sub", output="screen"
    )

    talker_node = fogros2.CloudNode(
        package="sam_ros",
        executable="sam_pub",
        output="screen",
        machine=machine1,
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
