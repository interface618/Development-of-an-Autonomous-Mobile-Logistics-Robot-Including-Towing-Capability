#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === Launch args ===
    channel_type      = LaunchConfiguration('channel_type')
    serial_port       = LaunchConfiguration('serial_port')
    serial_baudrate   = LaunchConfiguration('serial_baudrate')
    # frame_id는 문자열이라 네임스페이스 자동 적용 X → 기본값을 아예 'shark1/laser_link'로
    frame_id          = LaunchConfiguration('frame_id')
    inverted          = LaunchConfiguration('inverted')
    angle_compensate  = LaunchConfiguration('angle_compensate')
    scan_mode         = LaunchConfiguration('scan_mode')
    use_sim_time      = LaunchConfiguration('use_sim_time')

    

    return LaunchDescription([
        DeclareLaunchArgument('channel_type',     default_value='serial',
                              description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port',      default_value='/dev/ttyUSB1',
                              description='Serial port of lidar'),
        DeclareLaunchArgument('serial_baudrate',  default_value='256000',
                              description='Baudrate for lidar'),
        # ★ 기본값을 shark1/laser_link로 박아둠
        DeclareLaunchArgument('frame_id',         default_value='shark1/laser_link',
                              description='frame_id of lidar'),
        DeclareLaunchArgument('inverted',         default_value='false',
                              description='Invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value='true',
                              description='Enable angle compensation'),
        DeclareLaunchArgument('scan_mode',        default_value='Sensitivity',
                              description='Scan mode'),
        DeclareLaunchArgument('use_sim_time',     default_value='false',
                              description='Use simulation clock'),

        # --- SLLIDAR 노드 ---
        Node(
            namespace='shark1',                         # ★ 네임스페이스 고정
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'use_sim_time':      use_sim_time,
                'channel_type':      channel_type,
                'serial_port':       serial_port,
                'serial_baudrate':   serial_baudrate,
                'frame_id':          frame_id,          # ★ 'shark1/laser_link'
                'inverted':          inverted,
                'angle_compensate':  angle_compensate,
                'scan_mode':         scan_mode
            }],
            
        ),

        Node(
            package='sllidar_ros2',
            executable='rear_half_scan',
            name='rear_half_scan',
            output='screen'
        )
        

        
    ])
