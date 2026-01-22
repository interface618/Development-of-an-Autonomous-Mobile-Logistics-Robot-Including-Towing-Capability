import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # (이 부분은 현재 사용되지 않지만 그대로 둡니다)
    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters1.yaml'
    )

    return LaunchDescription([

        # 1. 가장 먼저 stella_ahrs_node 실행
        Node(
            package='stella_ahrs',
            executable='stella_ahrs_node',
            name='ahrs_node',
            output='screen',
            emulate_tty=True,
            namespace='/'
        ),

        # 2. 7초 후 usb_cam_node_exe 실행
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='usb_cam',
                    executable='usb_cam_node_exe',
                    name='usb_cam',
                    output='screen',
                    namespace='shark1/'
                )
            ]
        ),

        # 3. 첫 번째 aruco_node 실행 (DICT_5X5_250)
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_detector', # 첫 번째 노드 이름
            output='screen',
            parameters=[{
                'marker_size': 0.05,
                'aruco_dictionary_id': 'DICT_5X5_250',
                'image_topic': 'shark1/image_raw',
                'camera_info_topic': 'shark1/camera_info'
            }]
        ),
        
        # 4. 새로 추가된 두 번째 aruco_node 실행 (DICT_4X4_50)
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_detector_4x4', # 노드 이름이 겹치지 않게 변경!
            namespace='/aruco_4x4',     # --ros-args -r __ns:=/aruco_4x4
            output='screen',
            parameters=[{
                'aruco_dictionary_id': 'DICT_4X4_50', # -p aruco_dictionary_id
                'marker_size': 0.03,                 # -p marker_size
                'image_topic': '/shark1/image_raw',         # -p image_topic
                'camera_info_topic': '/shark1/camera_info'  # -p camera_info_topic
            }]
        )

    ])
