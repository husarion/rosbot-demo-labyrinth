from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    maze_cam = get_package_share_directory('maze_cam')
    video_device = os.environ.get('VIDEO_DEVICE')

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[
                maze_cam + '/config/v4l2_camera_params.yaml',
                {'video_device': video_device}
            ]
        ),

        Node(
            package='maze_cam',
            executable='get_image',
            name='get_image',
            parameters=[{'camera_topic': 'image_raw'}]
        ),

])
