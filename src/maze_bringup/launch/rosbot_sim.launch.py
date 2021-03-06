import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    world_file_name =  'maze_sim.world'

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'urdf/rosbot.urdf'
    urdf = os.path.join(
        get_package_share_directory('maze_bringup'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    gazebo_ros = get_package_share_directory('gazebo_ros')
    maze_bringup = get_package_share_directory('maze_bringup')

    gazebo_client = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
                condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    spawn_rosbot = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(maze_bringup, 'launch', 'rosbot_spawn.launch.py'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(maze_bringup, 'worlds', world_file_name), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('gdb', default_value='false',
                              description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('state', default_value='true',
                                description='Set "false" not to load "libgazebo_ros_state.so"'),
        gazebo_server,
        gazebo_client,
        spawn_rosbot,
    ])

if __name__ == '__main__':
    generate_launch_description()
