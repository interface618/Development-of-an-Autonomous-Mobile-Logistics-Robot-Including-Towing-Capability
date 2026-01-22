#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatesListener(Node):
    def __init__(self):
        super().__init__('joint_states_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def joint_states_callback(self, msg):
        motor_degrees = ['2']
        for i in range(min(6, len(msg.position))):  # 배열 길이 체크
            format_degree = f"{msg.position[i]:.6f}"
            motor_degrees.append(format_degree)

        self.get_logger().info(f"Position: {motor_degrees}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
