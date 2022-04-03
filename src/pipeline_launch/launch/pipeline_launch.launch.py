
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

def generate_launch_description():
    rpi_stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rpi_stereo'),
                'launch/rpi_stereo_with_param.launch.py'))
    )
    anynet_disparity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('anynet_disparity'),
                'launch/anynet_disparity.launch.py'))
    )
    return LaunchDescription([
        rpi_stereo_launch,
        anynet_disparity_launch 
    ])


