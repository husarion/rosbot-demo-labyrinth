from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0","0","0","0","0","0","map","odom",]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0","0","0","0","0","0","odom","base_link",]
        ),

    ])