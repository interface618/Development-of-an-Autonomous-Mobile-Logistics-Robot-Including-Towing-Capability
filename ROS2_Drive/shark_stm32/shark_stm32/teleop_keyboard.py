#!/usr/bin/env python3
import os
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# === 사용자 로봇에 맞게 직접 설정 ===
MAX_LIN_VEL = 0.5   # m/s 단위, 원하는 최대 직진 속도
MAX_ANG_VEL = 2.0   # rad/s 단위, 원하는 최대 회전 속도
LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1


msg = """
Control Your Robot with Keyboard!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key : force stop

CTRL-C to quit
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def constrain(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/shark1/cmd_vel', 10)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

    def update_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing: linear={linear:.2f}, angular={angular:.2f}')


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopKeyboard()

    print(msg)

    try:
        while True:
            key = get_key(settings)
            if key == 'w':
                node.target_linear_vel = constrain(node.target_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            elif key == 'x':
                node.target_linear_vel = constrain(node.target_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            elif key == 'a':
                node.target_angular_vel = constrain(node.target_angular_vel + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
            elif key == 'd':
                node.target_angular_vel = constrain(node.target_angular_vel - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
            elif key == ' ' or key == 's':
                node.target_linear_vel = 0.0
                node.target_angular_vel = 0.0
            elif key == '\x03':  # Ctrl-C
                break

            node.update_velocity(node.target_linear_vel, node.target_angular_vel)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot when exiting
        node.update_velocity(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
