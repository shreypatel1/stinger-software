'''
This launches the robot localization package
This fuses the IMU and GPS data
This publishes topic /odom
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('stinger_bringup')

    robot_localization_file_path = (pkg_share + '/config/ekf.yaml')
    navsat_transform_file_path = (pkg_share + '/config/navsat_transform.yaml')

    # URDF File Path
    xacro_file = os.path.join(
        get_package_share_directory('stinger_description'),
        'urdf',
        'stinger_tug.urdf.xacro'
    )
    
    # Get URDF from xacro
    robot_description = xacro.process(xacro_file)

    return LaunchDescription([
        # Robot description publisher
        Node(
            name = 'robot_state_publisher',
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            output = 'screen',
            parameters = [{'robot_description': robot_description}]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[robot_localization_file_path],
        ),    
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_transform_file_path],
            respawn=True,
            remappings=[
            # TODO: 4.4.b Navsat Node
            # Example: (topic, remaped_topic)
            ### STUDENT CODE HERE
            ('/imu', '/stinger/imu/relative'),
            ('/gps/fix', '/stinger/gps/fix')
            ### END STUDENT CODE
            ],
        ),
        Node(
            package='stinger_bringup',
            executable='imu_republisher',
            name='imu_republisher',
            output='screen'
        )
    ])
