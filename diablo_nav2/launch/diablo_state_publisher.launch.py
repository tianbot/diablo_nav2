import os

import launch
import launch.substitutions
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('diablo_simulation')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'diablo_simulation.urdf.xacro')
    robot_desc = launch.substitutions.Command('xacro %s' % xacro_file)
    params = {'robot_description': robot_desc}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[params])

    return launch.LaunchDescription([rsp])