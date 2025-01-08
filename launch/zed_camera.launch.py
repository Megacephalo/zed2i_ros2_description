#!/usr/bin/env python3

from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare('zed2i_ros2_description').find('zed2i_ros2_description')
    xacro_file_path = f'{pkg_path}/model/zed2i_urdf.xacro'

    assert Path(xacro_file_path).exists(), f'URDF file not found at {xacro_file_path}'
    assert Path(xacro_file_path).is_file(), f'URDF file {xacro_file_path} is NOT a file'

    print(f'Reading xacro file: {xacro_file_path}')

    task_queue = []

    task_queue.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_file_path])}],
        )
    )

    task_queue.append(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output="screen"
        )
    )

    task_queue.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # output='screen'
        )
    )

    return LaunchDescription(task_queue)
