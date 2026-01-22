#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import subprocess

class RosoutTrigger(Node):
    def __init__(self):
        super().__init__('rosout_trigger')

        # --- 트리거 조건 파라미터 ---
        self.declare_parameter('target_node_name', 'bt_navigator')  # Log.name
        self.declare_parameter('target_level', 20)                  # INFO=20
        self.declare_parameter('target_substring', 'Goal succeeded') # Log.msg 포함

        # --- 실행 커맨드 ---
        # launch 파일이나 터미널에서 이 파라미터 값을 아래 형식으로 지정해주세요.
        # 예: bash -c "source ~/catkin_ws/install/setup.bash && ros2 run my_pkg my_node"
        self.declare_parameter('run_cmd', 'echo "Please set run_cmd parameter"')

        # 중복 실행 방지
        self.triggered = False

        # /rosout 구독
        self.sub = self.create_subscription(Log, '/rosout', self._on_log, 100)

    def _on_log(self, msg: Log):
        if self.triggered:
            return

        target_node = self.get_parameter('target_node_name').get_parameter_value().string_value
        target_level = self.get_parameter('target_level').get_parameter_value().integer_value
        target_substr = self.get_parameter('target_substring').get_parameter_value().string_value

        # 조건: 노드명, 레벨, 메시지 부분일치
        if (msg.name == target_node) and (msg.level == target_level) and (target_substr in msg.msg):
            run_cmd = self.get_parameter('run_cmd').get_parameter_value().string_value
            self.get_logger().info(f'Match detected. Launching: {run_cmd}')
            try:
                # [수정된 부분]
                # shlex.split 대신 shell=True를 사용하여 'source'나 '&&' 같은
                # 쉘 스크립트 문법을 포함한 명령어를 실행합니다.
                subprocess.Popen(run_cmd, shell=True, executable='/bin/bash')
                
                self.triggered = True
                self.get_logger().info('Triggered process launched (once).')
            except Exception as e:
                self.get_logger().error(f'Failed to launch: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RosoutTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
