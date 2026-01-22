#!/usr/bin/env python3

import rclpy
import serial
import time
import threading
import math
import sys
import select
from rclpy.node import Node
# ✨ 1. Float64 임포트 추가
from std_msgs.msg import String, Float64MultiArray, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

# --- 전역 변수 ---
ser = None
tx_stm32_cmd_vel = ''
tx_stm32_motor_degrees = ''
tx_stm32_hook = ''
tx_stm32_reset = ''
odom_data = None
robot_data = '0'

class STM32SerialNode(Node):
    def __init__(self):
        super().__init__('stm32_serial_node')
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('serial_baudrate', 115200)

        self.setup_serial()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.braccio_state_pub = self.create_publisher(Float64MultiArray, 'braccio_state', 10)
        self.hook_pub = self.create_publisher(Twist, 'hook_mode', 10)

        # ✨ 2. Publisher의 메시지 타입을 String에서 Float64로 변경
        self.l_lider_pub = self.create_publisher(Float64, 'l_lider', 10)
        self.r_lider_pub = self.create_publisher(Float64, 'r_lider', 10)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.create_subscription(Twist, 'hook_mode', self.hook_callback, 10)

        threading.Thread(target=self.send_data_thread, daemon=True).start()
        threading.Thread(target=self.receive_data_thread, daemon=True).start()
        threading.Thread(target=self.keyboard_input_thread, daemon=True).start()

        self.create_timer(1.0 / 20.0, self.timer_callback)
        self.get_logger().info("STM32 시리얼 노드가 네임스페이스 없이 시작되었습니다.")

    def setup_serial(self):
        global ser
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('serial_baudrate').value
        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=None)
            self.get_logger().info(f"{port}에 {baudrate} 보드레이트로 연결되었습니다.")
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 {port}를 열 수 없습니다: {e}")
            rclpy.shutdown()

    def receive_data_thread(self):
        global odom_data
        buffer = b''
        while rclpy.ok():
            if ser and ser.in_waiting > 0:
                try:
                    buffer += ser.read(ser.in_waiting)
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        received_data = line.decode('utf-8', errors='ignore').strip()

                        # ✨ 3. 데이터 처리 로직 수정 ✨
                        # '$L:' -> 오른쪽 Lider (/r_lider)
                        if received_data.startswith('$L:'):
                            try:
                                value_str = received_data.split(':')[1].strip()
                                if value_str:
                                    # 문자열을 float으로 변환
                                    value_float = float(value_str)
                                    # Float64 메시지 생성 및 데이터 할당
                                    r_msg = Float64()
                                    r_msg.data = value_float
                                    self.r_lider_pub.publish(r_msg)
                            except (IndexError, ValueError) as e:
                                self.get_logger().warn(f"'$L:' 형식 파싱 또는 변환 오류: '{received_data}', Error: {e}")

                        # '$R:' -> 왼쪽 Lider (/l_lider)
                        elif received_data.startswith('$R:'):
                            try:
                                value_str = received_data.split(':')[1].strip()
                                if value_str:
                                    # 문자열을 float으로 변환
                                    value_float = float(value_str)
                                    # Float64 메시지 생성 및 데이터 할당
                                    l_msg = Float64()
                                    l_msg.data = value_float
                                    self.l_lider_pub.publish(l_msg)
                            except (IndexError, ValueError) as e:
                                self.get_logger().warn(f"'$R:' 형식 파싱 또는 변환 오류: '{received_data}', Error: {e}")

                        elif '|' in received_data and '#' in received_data:
                            try:
                                odom_part, _, _ = received_data[1:].split('|', 2)
                                odom_data = odom_part.strip().split()
                            except ValueError:
                                self.get_logger().warn(f"Odometry 데이터 파싱 불가: {received_data}")

                except Exception as e:
                    self.get_logger().error(f"데이터 수신 쓰레드에서 오류 발생: {e}")
                    break
            time.sleep(0.01)

    # --- 이하 다른 함수들은 수정할 필요가 없습니다 ---
    
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

        cmd_vel.append(format_linear)
        cmd_vel.append(format_angular)
        tx_stm32_cmd_vel = ''.join(map(str, cmd_vel))

    def joint_states_callback(self, msg):
        global tx_stm32_motor_degrees
        motor_degrees = ['2']
        for i in range(min(6, len(msg.position))):
            format_degree = f"{msg.position[i]:.6f}"
            motor_degrees.append(format_degree)
        tx_stm32_motor_degrees = ''.join(map(str, motor_degrees))

    def hook_callback(self, msg):
        global tx_stm32_hook
        hook = ['3']
        if msg.linear.x == 1.0:
            hook = ['4']
        if msg.linear.x == 2.0:
            hook = ['5']
        tx_stm32_hook = ''.join(map(str, hook))

    def send_command(self, command):
        if ser and ser.is_open and command:
            command_with_newline = command + '\n'
            ser.write(command_with_newline.encode())
            time.sleep(0.05)

    def send_data_thread(self):
        global tx_stm32_cmd_vel, tx_stm32_motor_degrees, tx_stm32_hook, tx_stm32_reset
        while rclpy.ok():
            try:
                self.send_command(tx_stm32_cmd_vel)
                self.send_command(tx_stm32_motor_degrees)
                self.send_command(tx_stm32_hook)
                if tx_stm32_reset == '3':
                    self.send_command(tx_stm32_reset)
                    tx_stm32_reset = ''
            except Exception as e:
                self.get_logger().warn(f"데이터 전송 쓰레드에서 오류 발생: {e}")
            time.sleep(0.1)

    def keyboard_input_thread(self):
        global tx_stm32_reset
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 'r':
                    self.get_logger().info("키보드로 RESET 신호 수신!")
                    tx_stm32_reset = '3'

    def timer_callback(self):
        self.pub_odometry()
        self.pub_braccio_state()
        self.pub_hook()

    def pub_hook(self):
        hook_msg = Twist()
        hook_msg.linear.x = 0.0
        self.hook_pub.publish(hook_msg)

    def pub_odometry(self):
        global odom_data
        if odom_data is not None:
            try:
                odom_data_float = [float(val) if val != 'nan' else 0.0 for val in odom_data]
                if len(odom_data_float) < 6: return

                x, y, th, dt, vx, vy = odom_data_float[:6]
                now = self.get_clock().now().to_msg()
                quaternion = quaternion_from_euler(0, 0, th)
                
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = "odom"
                t.child_frame_id = "base_footprint"
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
                self.tf_broadcaster.sendTransform(t)

                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_footprint"
                odom_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
                odom_msg.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
                odom_msg.twist.twist.linear.x = vx
                odom_msg.twist.twist.angular.z = vy
                self.odom_pub.publish(odom_msg)
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"Odometry 발행 오류: {e}")

    def pub_braccio_state(self):
        global robot_data
        braccio_msg = Float64MultiArray()
        try:
            braccio_msg.data = [float(i) for i in robot_data]
            self.braccio_state_pub.publish(braccio_msg)
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
