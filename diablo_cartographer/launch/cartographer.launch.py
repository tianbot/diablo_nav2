from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** File paths ******
    pkg_share = FindPackageShare('diablo_cartographer').find('diablo_cartographer')


    ## ***** Nodes *****

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', pkg_share + '/config',
            '-configuration_basename', 'diablo_2d.lua'],
         remappings = [
             ('scan', '/scan'),
             ('imu','/livox/imu')],
        output = 'screen'
        )

    rviz_node=Node(
        package = 'rviz2',
        namespace = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen' ,
        arguments = ['-d', pkg_share + '/rviz/diablo_2d.rviz']
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )
    tf2_node = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0','0','0','0','0','0','base_footprint','livox_frame']
    )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        tf2_node
    ])
