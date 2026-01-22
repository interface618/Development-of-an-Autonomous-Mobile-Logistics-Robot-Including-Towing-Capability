import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 이 launch 파일이 속한 패키지의 이름을 넣어주세요.
    pkg_name = 'aruco_imu_aligner' 

    # 파라미터 파일의 전체 경로를 찾습니다.
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'aligner_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='aruco_imu_aligner', # ros2 run에서 사용한 실행파일 이름
            name='aruco_aligner',           # 노드에 부여할 이름 (파라미터 파일과 일치)
            parameters=[params_file],       # 로드할 파라미터 파일 지정
            output='screen'                 # 노드의 로그를 터미널에 바로 출력
        ),
    ])
