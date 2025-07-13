'''
This launches the bringup of IMU, GPS, camera, and LiDAR.
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='stinger_controller',
            executable='throttle_controller',
            output='screen',
            name='throttle_controller',
        ),
        Node(
            package='stinger_controller',
            executable='acceleration_controller',
            output='screen',
            name='acceleration_controller',
            remappings=[
                ('/imu/data', '/stinger/imu/data')
            ]
        ),
    ])
