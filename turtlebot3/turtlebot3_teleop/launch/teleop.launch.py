#!/usr/bin/env python3


#######################################
### This is for BringUp aedbot_nodes###
#######################################

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions

def generate_launch_description():
    package_name = 'turtlebot3_teleop'  # 패키지 이름을 여기에 입력해주세요

    actions = []

    # 특정 Python 파일들의 실행을 추가합니다.

    actions.append(launch_ros.actions.Node(
        package=package_name,
        executable='teleop_keyboard',
        output='screen',
        prefix = 'xterm -e',
        name='teleop_keyboard'
    ))

    return LaunchDescription(actions)

if __name__ == '__main__':
    generate_launch_description()