from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
  
def generate_launch_description():
    maze_bringup = get_package_share_directory('maze_bringup')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame'],
            parameters=[
        		maze_bringup + '/config/static_tf.yaml'
        	],
            ),

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dummy'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.2', '0', '0.8', '0', '0', '0', 'base_link', 'camera_rgb_frame'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_fl'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_fr'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_rl'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_rr'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.03', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[
        		maze_bringup + '/config/static_tf.yaml'
            ]
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_link', 'laser'],
            parameters=[
        		maze_bringup + '/config/static_tf_sim.yaml'
            ]
            ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-spawn_service_timeout', '60', '-entity', 'rosbot', '-x', '0', '-y', '0', '-z', '0.03', '-file', maze_bringup + '/models/rosbot.sdf']),
    ])
