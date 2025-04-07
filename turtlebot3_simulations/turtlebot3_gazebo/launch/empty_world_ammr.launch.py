#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 설정
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # world 파일과 xacro 파일 경로 지정
    world_file = os.path.join(turtlebot3_gazebo_pkg, 'worlds', 'empty_world.world')
    xacro_file = os.path.join(turtlebot3_gazebo_pkg, 'urdf', 'ammr.urdf.xacro')
    sdf_file = os.path.join(turtlebot3_gazebo_pkg, 'urdf', 'ammr.sdf')
    # xacro_file = os.path.join(turtlebot3_gazebo_pkg, "urdf", "ammr.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": use_sim_time}

    # 런치 구성 변수
    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # gzserver 실행 (empty world)
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # gzclient 실행 (GUI)
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    # robot_state_publisher 실행 (TF 브로드캐스트)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # spawn_entity 노드를 이용해 "ammr" 로봇 스폰
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ammr',
            '-file', sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # 런치 인자 선언
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='로봇의 초기 X 위치'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='로봇의 초기 Y 위치'
    ))
    ld.add_action(DeclareLaunchArgument(
        'z_pose', default_value='0.01',
        description='로봇의 초기 Z 위치'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='시뮬레이션 시간 사용 여부'
    ))

    # 각 노드 추가
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(robot_state_publisher_node)
    # gzserver가 완전히 실행된 후 spawn_entity를 실행하도록 딜레이 추가 (5초)
    ld.add_action(spawn_robot)

    return ld
