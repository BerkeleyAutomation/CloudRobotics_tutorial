from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', 
            default_value='robot4',
            description='Cloudgripper robot name'
        ),
        Node(
            package='cloudgripper_ros',
            executable='robot_subscriber',
            name='robot_subscriber',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
        Node(
            package='cloudgripper_ros',
            executable='image_service',
            name='image_service',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
        Node(
            package='cloudgripper_ros',
            executable='state_service',
            name='state_service',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
        Node(
            package='cloudgripper_ros',
            executable='keyboard_teleop_control',
            name='keyboard_teleop_control',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
        Node(
            package="fogros2_tutorial", 
            executable="fog_rtx_recorder", 
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ), 
        Node(
            package="fogros2_tutorial", executable="sam_client",
        )
    ])