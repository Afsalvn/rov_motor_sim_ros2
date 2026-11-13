"""Start the bridge node and the motor simulator with config/params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # installed copy of the yaml (see data_files in setup.py)
    params_file = os.path.join(
        get_package_share_directory('rov_motor_sim'),
        'config',
        'params.yaml',
    )

    # node names have to match the top-level keys in params.yaml
    bridge_node = Node(
        package='rov_motor_sim',
        executable='bridge_node',
        name='bridge_node',
        parameters=[params_file],
        output='screen',
    )

    motor_sim_node = Node(
        package='rov_motor_sim',
        executable='motor_sim_node',
        name='motor_sim_node',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([bridge_node, motor_sim_node])
