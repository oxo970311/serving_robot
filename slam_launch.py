from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_pkg')

    slam_params = os.path.join(
        pkg_share,
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',               # ✅ 여기!
            executable='sync_slam_toolbox_node',  # ✅ slam_toolbox 실행 파일
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        )
    ])