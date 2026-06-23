#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    finenav_demo_pkg_dir = get_package_share_directory('finenav_demo')
    mppi_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'mppi_params.yaml'
    )
    finenav_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'finenav_params.yaml'
    )
    obs_cov_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'obs_cov.yaml'
    )

    start_hello_finenav = Node(
        package='finenav_demo',
        executable='hello_finenav',
        output='screen',
        parameters=[
            {'use_sim_time': False,},
            mppi_params_file,
            finenav_params_file,
            obs_cov_file,
        ],
    )

    start_goal_pose_bridge = Node(
        package='finenav_demo',
        executable='goal_pose_bridge_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ]
    )

    velocity_smoother_pkg_dir = get_package_share_directory('velocity_smoother')
    smoother_params_file = os.path.join(
        velocity_smoother_pkg_dir,
        'config',
        'velocity_smoother_params.yaml'
    )

    start_velocity_smoother = Node(
        package='velocity_smoother',
        executable='velocity_smoother_node',
        output='screen',
        parameters=[smoother_params_file],
    )

    ld = LaunchDescription()
    ld.add_action(start_hello_finenav)
    ld.add_action(start_goal_pose_bridge)
    ld.add_action(start_velocity_smoother)

    return ld
