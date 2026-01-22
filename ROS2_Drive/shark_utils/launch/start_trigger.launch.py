# shark_utils/launch/start_trigger.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 방금 만든 파라미터 파일의 전체 경로를 찾습니다.
    params_file = os.path.join(
        get_package_share_directory('shark_utils'),
        'config',
        'trigger_params.yaml'
    )

    # 2. rosout_trigger 노드를 실행하되, parameters 옵션으로 파일을 지정합니다.
    trigger_node = Node(
        package='shark_utils',
        executable='rosout_trigger', # .py 확장자는 생략
        name='rosout_trigger',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        trigger_node
    ])
