from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cando_base',
            namespace='rpi_stereo_cam',
            executable='rpi_stereo_main',
            name='rpi_stereo'
        ),
    ])
