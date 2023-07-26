from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('diablo_navigation2'),
        'config',
        'diablo_ekf.yaml'
        ),
    
    node=Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config],
            remappings = [('odometry/filtered','odom')]
           )
    ld.add_action(node)
    return ld
