#!/usr/bin/env python3

import rclpy
import serial
import time
import threading
import math
import sys
import select
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

ser = None

tx_stm32_cmd_vel = ''
tx_stm32_motor_degrees = ''
tx_stm32_hook = ''
tx_stm32_reset = ''

pre_th = 0
odom_data = None
robot_data = '0'
l_lider = '0' # 레이더의 값을 0으로 하면 오류가 날것이기에 255를 넣음
r_lider = '0'

class STM32SerialNode(Node):
    def __init__(self):
        super().__init__('stm32_serial', namespace='shark1')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)

        self.setup_serial()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.braccio_state_pub = self.create_publisher(Float64MultiArray, 'braccio_state', 10)
        self.hook_pub = self.create_publisher(Twist, 'hook_mode', 10)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.create_subscription(Twist, 'hook_mode', self.hook_callback, 10)

        threading.Thread(target=self.send_data_thread, daemon=True).start()
        threading.Thread(target=self.receive_data_thread, daemon=True).start()
        threading.Thread(target=self.keyboard_input_thread, daemon=True).start()

        self.create_timer(1.0 / 20.0, self.timer_callback)

    def setup_serial(self):
        global ser
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('serial_baudrate').value
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=None)
        self.get_logger().info(f"Connected to {port} at {baudrate} baud rate.")

    def cmd_vel_callback(self, msg):
        global tx_stm32_cmd_vel
        cmd_vel = ['1']
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x < 0:
            linear_x = abs(linear_x)
        else:
            linear_x += 100

        if angular_z < 0:
            angular_z = abs(angular_z)
        else:
            angular_z += 100

        format_linear = f"{linear_x:03.6f}"
        format_angular = f"{angular_z:03.6f}"

        if linear_x < 100:
            format_linear = f"00{format_linear}"
        if angular_z < 100:
            format_angular = f"00{format_angular}"

        cmd_vel.append(format_linear)
        cmd_vel.append(format_angular)

        tx_stm32_cmd_vel = ''.join(map(str, cmd_vel))

    def joint_states_callback(self, msg):
        global tx_stm32_motor_degrees
        motor_degrees = ['2']
        for i in range(6):
            format_degree = f"{msg.position[i]:.6f}"
            motor_degrees.append(format_degree)
        tx_stm32_motor_degrees = ''.join(map(str, motor_degrees))
        
    
    def hook_callback(self, msg):
        global tx_stm32_hook
        #print(msg.linear.x)

        hook = ['3']             # 가만히

        if msg.linear.x == 1.0 : # 올리기
            hook = ['4']
        
        if msg.linear.x == 2.0 : # 내리기
            hook = ['5']

        tx_stm32_hook = ''.join(map(str, hook))

    def send_command(self, command):
        command += '\n'
        ser.write(command.encode())
        time.sleep(0.05)

    def send_data_thread(self):
        global tx_stm32_cmd_vel, tx_stm32_motor_degrees, tx_stm32_reset
        while rclpy.ok():
            try:
                self.send_command(tx_stm32_cmd_vel)
                self.send_command(tx_stm32_motor_degrees)
                self.send_command(tx_stm32_hook)
                if tx_stm32_reset == '3':
                    self.send_command(tx_stm32_reset)
                    tx_stm32_reset = ''
            except TimeoutError:
                self.get_logger().warn("Timeout Error!!! When Send Data")

    def receive_data_thread(self):
        global odom_data, robot_data, l_lider, r_lider
        buffer = b''
        while rclpy.ok():
            if ser.in_waiting > 0:
                buffer = ser.read_until(b'\r\n')
                received_data = buffer.decode('utf-8', errors='ignore').strip()

                if '|' in received_data and '#' in received_data : # 이전 데이터 부분
                    try:
                        odom_part, l_lider_part, r_lider_part = received_data[1:].split('|', 2)
                        odom_data = odom_part.strip().split()
                        r_lider = r_lider_part.strip().split()
                        l_lider = l_lider_part.strip().split()
                        print("R: ", r_lider)
                        print("L: ", l_lider)

                    except UnicodeDecodeError as e:
                        self.get_logger().warn(f"Decoding error: {e}")
                buffer = b''

    def keyboard_input_thread(self):
        global tx_stm32_reset
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == 'r':
                    print("RESET!")
                    tx_stm32_reset = '3'

    def timer_callback(self):
        self.pub_odometry()
        self.pub_braccio_state()
        self.pub_hook()

    def pub_hook(self):
        hook_msg = Twist()
        hook_msg.linear.x = 0.0 # 후크 조정 부분 나중에 정렬 코드로 이전될꺼임.
        self.hook_pub.publish(hook_msg)

    def pub_odometry(self):
        global odom_data
        if odom_data is not None:
            odom_data_float = [float(val) if val != 'nan' else 0.0 for val in odom_data]
            x, y, th, dt, vx, vy = odom_data_float[:6]
            quaternion = quaternion_from_euler(0, 0, th)

            odom_msg = Odometry()
            now = self.get_clock().now().to_msg()
            odom_msg.header.stamp = now
            odom_msg.header.frame_id = "shark1/odom"
            odom_msg.child_frame_id = "shark1/base_footprint"

            odom_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
            odom_msg.pose.pose.orientation = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )

            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.z = vy

            odom_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

            self.odom_pub.publish(odom_msg)
            
            # ROS2 방식 TF 발행
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "shark1/odom"
            t.child_frame_id = "shark1/base_footprint"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            self.tf_broadcaster.sendTransform(t)
            
    def pub_braccio_state(self):
        global robot_data
        braccio_msg = Float64MultiArray()
        braccio_msg.data = [float(i) for i in robot_data]
        self.braccio_state_pub.publish(braccio_msg)

def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
