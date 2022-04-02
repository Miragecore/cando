import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from pathlib import Path

config_path = Path(get_package_share_directory('rpi_stereo'), 'config')

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    l_camera_info_file_arg = DeclareLaunchArgument(
        "l_camera_info_file", default_value=TextSubstitution(text=str(Path(config_path, "left.yaml")))
    )
    r_camera_info_file_arg = DeclareLaunchArgument(
        "r_camera_info_file", default_value=TextSubstitution(text=str(Path(config_path, 'right.yaml')))
    )
    rectify_arg = DeclareLaunchArgument(
        "rectify", default_value='True'
    )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    rpi_stereo_node = Node(
            package='rpi_stereo',
            namespace='rpi_stereo',
            executable='rpi_stereo_node',
            name='rpi_stereo',
            parameters=[{
                "l_camera_info_file": LaunchConfiguration('l_camera_info_file'),
                "r_camera_info_file": LaunchConfiguration('r_camera_info_file'),
                "rectify": LaunchConfiguration('rectify'),
            }]
        )

    return LaunchDescription([
        l_camera_info_file_arg,
        r_camera_info_file_arg,
        rectify_arg,
        rpi_stereo_node,
    ])
