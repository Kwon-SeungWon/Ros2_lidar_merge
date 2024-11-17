from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # 시뮬레이션 시간 사용
                'odom_frame': 'odom',
                'base_link_frame': 'base_footprint',
                'world_frame': 'odom',
                'publish_tf': True,
                'frequency': 50.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,  # 2D 모드 활성화 (yaw만 계산)
                'odom0': '/odom',
                'odom0_config': [True, True, False, False, False, True,  # 위치 (x, y) 및 yaw 사용
                                 False, False, False, False, False, False],
                'imu0': '/imu',
                'imu0_config': [False, False, False, True, True, True,  # 각속도 (roll, pitch, yaw)
                                False, False, False, True, True, True], # 가속도 사용
                'imu0_differential': False,
                'imu0_remove_gravitational_acceleration': True,
                # 'process_noise_covariance': [  # EKF 상태 벡터에 대한 노이즈
                #     1e-4, 0, 0,   0, 0, 0,   # x, y, z 위치
                #     0, 1e-4, 0,   0, 0, 0,   # x, y, z 속도
                #     0, 0, 1e-3,   0, 0, 0,   # x, y, z 가속도
                #     0, 0, 0,   1e-2, 0, 0,   # roll, pitch, yaw
                #     0, 0, 0,   0, 1e-2, 0,   # 각속도 (roll, pitch, yaw)
                #     0, 0, 0,   0, 0, 1e-2    # 기타 상태
                # ],
                # 'initial_estimate_covariance': [  # 초기 상태 추정치에 대한 신뢰도
                #     1e-1, 0, 0,   0, 0, 0,
                #     0, 1e-1, 0,   0, 0, 0,
                #     0, 0, 1e-1,   0, 0, 0,
                #     0, 0, 0,   1e-2, 0, 0,
                #     0, 0, 0,   0, 1e-2, 0,
                #     0, 0, 0,   0, 0, 1e-2
                # ],
            }]
        ),
    ])
