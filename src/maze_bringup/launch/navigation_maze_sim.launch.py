from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    maze_bringup = get_package_share_directory('maze_bringup')
    
    
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([maze_bringup, '/launch/rosbot_sim.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([maze_bringup, '/launch/rosbot_navigation_sim.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([maze_bringup, '/launch/transform.launch.py'])
        ),

        Node(
            package='maze_robot',
            executable='robot_controller'
        ),      

        Node(
            package='maze_robot',
            executable='camera_sim',
            parameters=[
                maze_bringup + '/config/camera_params_sim.yaml'
            ]
        ),  

        Node(
            package='rviz2',
            executable='rviz2',
            name="rviz2",
            arguments=['-d', maze_bringup + "/config/rosbot.rviz"],
        ),

    ])