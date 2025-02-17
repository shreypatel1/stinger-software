"""
This launch file is a static tf publisher.


TODO: make sure urdf is there
"""

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():

    description_share = get_package_share_directory('stinger_description')

    urdf_path = os.path.join(description_share, 'urdf', 'stinger_tug.urdf') #'stinger_tug.urdf.xacro'
    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    # Robot description publisher
    robot_state_publisher = Node(
      name = 'robot_state_publisher',
      package = 'robot_state_publisher',
      executable = 'robot_state_publisher',
      output = 'screen',
      parameters = [{'robot_description': robot_desc}]
    )

    odom_to_map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments = ["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0.0", "--frame-id", "odom", "--child-frame-id", "map"]
        )

    return LaunchDescription([
        robot_state_publisher,
        odom_to_map_tf
    ])
