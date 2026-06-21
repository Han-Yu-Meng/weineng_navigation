# Copyright (c) 2026.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("velocity_smoother")
    config_path = os.path.join(pkg_dir, "config", "velocity_smoother_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="velocity_smoother",
                executable="velocity_smoother_node",
                name="velocity_smoother",
                output="screen",
                parameters=[config_path],
            ),
        ]
    )
