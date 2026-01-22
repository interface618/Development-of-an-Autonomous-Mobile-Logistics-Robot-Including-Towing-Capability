#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import serial
import time
import threading
import math
import sys
import select
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

# -----------------------------
# 전역(모듈 스코프) 변수
# -----------------------------
ser = None

tx_stm32_cmd_vel = ''     # STM32로 보낼 속도 명령(문자열 프레임)
tx_stm32_reset   = ''     # '3' 전송 시 리셋 커맨드

odom_data = None
distance_left = None
distance_right = None
limit_switch = None  # 0=주행, 1=견인


def _encode_cmd(vx: float, wz: float) -> str:
    # 프레임 헤더: '1'
    if vx < 0:
        lin = abs(vx)
    else:
        lin = vx + 100.0
    if wz < 0:
        ang = abs(wz)
    else:
        ang = wz + 100.0

    fl = f"{lin:03.6f}"
    fa = f"{ang:03.6f}"
    if lin < 100:
        fl = f"00{fl}"
    if ang < 100:
        fa = f"00{fa}"
    return '1' + fl + fa


class STM32SerialNode(Node):
    def __init__(self):
        super().__init__('stm32_serial', namespace='shark1')

        # 파라미터
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)
        # 토픽명(원하면 바꿀 수 있도록)
        self.declare_parameter('drive_cmd_topic', 'cmd_vel')            # /shark0/cmd_vel
        self.declare_parameter('traction_cmd_topic', 'cmd_vel_switch')  # /shark0/cmd_vel_switch

        # 모드 상태
        self.is_traction = False     # False=주행, True=견인
        self._last_mode = None

        # 시리얼
        self.setup_serial()

        # TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

        # 퍼블리셔
        self.odom_pub       = self.create_publisher(Odometry, 'odom', 10)
        self.dist_left_pub  = self.create_publisher(Float64, 'distance_left', 10)
        self.dist_right_pub = self.create_publisher(Float64, 'distance_right', 10)
        self.switch_pub     = self.create_publisher(Int32,   'limit_switch', 10)
        self.raw_pub        = self.create_publisher(String,  'stm32_raw', 10)

        # 서브스크라이버(★ 두 입력을 모두 구독하고 게이트)
        drive_topic    = self.get_parameter('drive_cmd_topic').get_parameter_value().string_value
        traction_topic = self.get_parameter('traction_cmd_topic').get_parameter_value().string_value
        self.create_subscription(Twist, drive_topic,    self._drive_cmd_cb,    10)
        self.create_subscription(Twist, traction_topic, self._traction_cmd_cb, 10)
        self.get_logger().info(f"[STM32] drive from '{drive_topic}', traction from '{traction_topic}'")

        # /shark0/alignment : 로봇 정렬 명령 관련 토픽
        self.create_subscription(Bool, 'alignment', self.check_alignment, 10)

        # 스레드 & 타이머
        threading.Thread(target=self.send_data_thread,    daemon=True).start()
        threading.Thread(target=self.receive_data_thread, daemon=True).start()
        threading.Thread(target=self.keyboard_input_thread, daemon=True).start()
        self.create_timer(1.0 / 20.0, self.timer_callback)

    # ----------------- 시리얼 -----------------
    def setup_serial(self):
        global ser
        port     = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('serial_baudrate').value
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.0)
        ser.reset_input_buffer()
        self.get_logger().info(f"Connected to {port} at {baudrate} baud.")

    # ----------------- 모드 전환 처리 -----------------
    def _apply_mode(self, sw: int):
        """limit_switch 수신 값으로 모드 반영(0=주행, 1=견인). 전환 시 한 번 정지 프레임 송신."""
        self.is_traction = (sw == 1)
        if self._last_mode is None:
            self._last_mode = self.is_traction
            return
        if self._last_mode != self.is_traction:
            self.get_logger().info(f"[mode] switch to {'TRACTION(1)' if self.is_traction else 'DRIVING(0)'} → stop once")
            # 전환 순간 잔류 명령 끊기
            try:
                ser.write((_encode_cmd(0.0, 0.0) + '\n').encode())
            except Exception:
                pass
            self._last_mode = self.is_traction

    # ----------------- cmd_vel 콜백(게이트) -----------------
    def _drive_cmd_cb(self, msg: Twist):
        """주행용 입력; 견인 모드일 땐 무시"""
        if self.is_traction:
            return
        self._set_tx_from_twist(msg)

    def _traction_cmd_cb(self, msg: Twist):
        """견인용 입력; 주행 모드일 땐 무시"""
        if not self.is_traction:
            return
        self._set_tx_from_twist(msg)

    def _set_tx_from_twist(self, msg: Twist):
        global tx_stm32_cmd_vel
        tx_stm32_cmd_vel = _encode_cmd(msg.linear.x, msg.angular.z)

    # ----------------- 공통 송신 -----------------
    def send_command(self, command: str):
        if not command:
            return
        try:
            ser.write((command + '\n').encode())
            time.sleep(0.05)
        except Exception as e:
            self.get_logger().warn(f"TX error: {e}")

    def send_data_thread(self):
        global tx_stm32_cmd_vel, tx_stm32_reset
        while rclpy.ok():
            try:
                self.send_command(tx_stm32_cmd_vel)
                if tx_stm32_reset == '3':
                    self.send_command(tx_stm32_reset)
                    tx_stm32_reset = ''
            except TimeoutError:
                self.get_logger().warn("Timeout Error!!! When Send Data")
            except Exception as e:
                self.get_logger().warn(f"send_data_thread error: {e}")
            time.sleep(0.01)

    # ----------------- 수신/파서 -----------------
    def receive_data_thread(self):
        global odom_data, distance_left, distance_right, limit_switch
        buf = bytearray()
        last_immediate_pub = time.time()

        def fconv(s: str) -> float:
            try:
                v = float(s)
                return 0.0 if (math.isnan(v) or math.isinf(v)) else v
            except Exception:
                return 0.0

        def iconv(s: str) -> int:
            try:
                return int(float(s))
            except Exception:
                return 0

        while rclpy.ok():
            try:
                chunk = ser.read(256)
                if chunk:
                    buf.extend(chunk)

                while True:
                    idx_n = buf.find(b'\n')
                    idx_r = buf.find(b'\r')
                    cut = -1
                    if idx_n != -1 and idx_r != -1:
                        cut = min(idx_n, idx_r)
                    elif idx_n != -1:
                        cut = idx_n
                    elif idx_r != -1:
                        cut = idx_r
                    if cut == -1:
                        break

                    line = bytes(buf[:cut]).decode('utf-8', errors='ignore').strip()
                    del buf[:cut+1]
                    if not line:
                        continue
                    self.raw_pub.publish(String(data=line))
                    if not line.startswith('#'):
                        continue

                    payload = line[1:].strip()
                    if '|' not in payload:
                        continue
                    left, right = payload.split('|', 1)
                    f_tokens = [t for t in left.strip().split() if t]
                    r_tokens = [t for t in right.strip().split() if t]
                    if len(f_tokens) != 6 or len(r_tokens) != 3:
                        continue

                    x = fconv(f_tokens[0]); y = fconv(f_tokens[1]); th = fconv(f_tokens[2])
                    dtt = fconv(f_tokens[3])  # 미사용
                    v_meas = fconv(f_tokens[4]); w_meas = fconv(f_tokens[5])
                    odom_data = [x, y, th, v_meas, w_meas]

                    new_left  = iconv(r_tokens[0])
                    new_right = iconv(r_tokens[1])
                    new_sw    = iconv(r_tokens[2])

                    distance_left  = new_left
                    distance_right = new_right
                    limit_switch   = new_sw

                    # ★ 모드 반영
                    self._apply_mode(limit_switch)

                    # 즉시 발행(최대 50Hz)
                    now = time.time()
                    if now - last_immediate_pub > 0.02:
                        m_sw = Int32();  m_sw.data = int(limit_switch);      self.switch_pub.publish(m_sw)
                        m_dl = Float64(); m_dl.data = float(distance_left);  self.dist_left_pub.publish(m_dl)
                        m_dr = Float64(); m_dr.data = float(distance_right); self.dist_right_pub.publish(m_dr)
                        last_immediate_pub = now

                time.sleep(0.002)

            except Exception as e:
                self.get_logger().warn(f"RX error: {e}")
                time.sleep(0.01)

    # ----------------- 키보드 -----------------
    def keyboard_input_thread(self):
        global tx_stm32_reset
        while rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == 'r':
                        print("RESET!")
                        tx_stm32_reset = '3'
                time.sleep(0.01)
            except Exception:
                time.sleep(0.2)

    # ----------------- 타이머 -----------------
    def timer_callback(self):
        self.pub_odometry()
        self.pub_dist_switch()

    def pub_odometry(self):
        global odom_data
        if odom_data is None:
            return
        x, y, th, v_meas, w_meas = odom_data
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, th)

        odom_msg = Odometry()
        now = self.get_clock().now().to_msg()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "shark1/odom"
        odom_msg.child_frame_id  = "shark1/base_footprint"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom_msg.twist.twist.linear.x  = v_meas
        odom_msg.twist.twist.angular.z = w_meas
        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]
        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "shark1/odom"
        t.child_frame_id  = "shark1/base_footprint"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def pub_dist_switch(self):
        global distance_left, distance_right, limit_switch
        if distance_left is not None:
            m = Float64(); m.data = float(distance_left);  self.dist_left_pub.publish(m)
        if distance_right is not None:
            m = Float64(); m.data = float(distance_right); self.dist_right_pub.publish(m)
        if limit_switch is not None:
            m = Int32();  m.data = int(limit_switch);      self.switch_pub.publish(m)

    # ----------------- 정렬 여부 -----------------
    def check_alignment(self, msg: Bool):
        try:
            if not msg.data:
                self.get_logger().info("[alignment] FALSE → ALIGN_RETRY")
                self.send_command('4')
            else:
                self.get_logger().info("[alignment] TRUE → ACTUATOR_ON")
                self.send_command('5')
        except Exception as e:
            self.get_logger().warn(f"alignment_cb error: {e}")


# =============================
# 엔트리 포인트
# =============================
def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
