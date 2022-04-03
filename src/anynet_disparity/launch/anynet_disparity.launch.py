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

package_name = 'anynet_disparity'

config_path = Path(get_package_share_directory(package_name), 'config')

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    pretrained_arg = DeclareLaunchArgument(
        "pretrained", default_value=TextSubstitution(text=str(Path(config_path, "checkpoint.tar")))
    )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    rpi_stereo_node = Node(
            package= package_name,
            executable='anynet_disparity',
            name='anynet_disparity',
            parameters=[{
                "pretrained": LaunchConfiguration('pretrained'),
            }]
        )

    return LaunchDescription([
        pretrained_arg,
        rpi_stereo_node 
    ])
