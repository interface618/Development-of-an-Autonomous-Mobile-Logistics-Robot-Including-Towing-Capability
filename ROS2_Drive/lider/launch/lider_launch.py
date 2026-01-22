from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    이 함수는 'ros2 launch' 명령어가 실행될 때 호출됩니다.
    실행할 노드들의 목록과 설정을 정의합니다.
    """
    return LaunchDescription([
        Node(
            package='lider',
            # setup.py의 entry_points에 정의된 실행 파일 이름과 같아야 합니다.
            executable='stm32_serial_node',
            # rqt_graph 등에서 보일 노드의 실제 이름입니다.
            name='stm32_serial_node',
            # 노드의 출력을 터미널에 바로 표시해줍니다 (디버깅에 유용).
            output='screen',
            emulate_tty=True,
            # 파이썬 노드에 전달할 파라미터들을 설정합니다.
            parameters=[
                # 실제 연결된 시리얼 포트 이름으로 수정하세요 (예: '/dev/ttyUSB0').
                {'serial_port': '/dev/ttyACM1'},
                {'serial_baudrate': 115200}
            ]
        )
    ])

