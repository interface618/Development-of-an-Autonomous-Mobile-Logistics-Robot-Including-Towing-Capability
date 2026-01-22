#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('shark_multi')   # 마스크 파일을 둘 패키지
    keepout_yaml = os.path.join(pkg, 'config', 'filters.yaml')

    return LaunchDescription([
        # 2-1) 마스크 OccupancyGrid 발행
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[keepout_yaml],
        ),
        # 2-2) 필터 정보 서버 (type=0: Keepout)
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[keepout_yaml],
            # 이 노드는 기본으로 /costmap_filter_info 를 발행
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['filter_mask_server',
                            'costmap_filter_info_server']
            }]
        )

    ])
