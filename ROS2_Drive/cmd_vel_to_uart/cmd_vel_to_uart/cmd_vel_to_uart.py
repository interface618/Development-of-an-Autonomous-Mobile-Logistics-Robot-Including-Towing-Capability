#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import serial
import time
import threading
import re

# 예: "... | 93.000000 50.000000 0.000000 ..." → 93, 50만 추출
PAIR_REGEX = re.compile(r"\|\s*([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)")

class CmdVelToUart(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_uart')

        # ----- 파라미터 -----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)
        # 'offset100' 또는 'signbit'
        self.declare_parameter('protocol_mode', 'offset100')
        # '\n' 또는 '\r\n'
        self.declare_parameter('eol', '\r\n')
        # 하트비트 주기(Hz)
        self.declare_parameter('tx_rate_hz', 20.0)
        # 단위 변환 스케일(예: mm→m이면 0.001)
        self.declare_parameter('laser_scale', 1.0)

        self.ser = None
        self.last_cmd = ''

        # 레이저 퍼블리셔
        self.pub_l = self.create_publisher(Float64, 'l_lider', 10)
        self.pub_r = self.create_publisher(Float64, 'r_lider', 10)

        self.setup_serial()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 하트비트 타이머
        rate = float(self.get_parameter('tx_rate_hz').get_parameter_value().double_value)
        self.create_timer(1.0 / max(1.0, rate), self.heartbeat_send)

        # 수신 스레드 시작(라인 단위 파싱)
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_thread.start()

    def setup_serial(self):
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = int(self.get_parameter('serial_baudrate').get_parameter_value().integer_value)
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)  # 짧은 timeout
        self.get_logger().info(f"Connected to {port} at {baud} baud rate.")

    # -------- 기존 송신 로직 유지 --------
    def _build_frame_offset100(self, linear_x, angular_z):
        if linear_x < 0:
            linear_x = abs(linear_x)
        else:
            linear_x += 100
        if angular_z < 0:
            angular_z = abs(angular_z)
        else:
            angular_z += 100

        format_linear  = f"{linear_x:03.6f}"
        format_angular = f"{angular_z:03.6f}"
        if linear_x < 100:
            format_linear  = f"00{format_linear}"
        if angular_z < 100:
            format_angular = f"00{format_angular}"
        return '1' + format_linear + format_angular

    def _build_frame_signbit(self, linear_x, angular_z):
        sign_x = '1' if linear_x >= 0 else '0'
        sign_z = '1' if angular_z >= 0 else '0'
        abs_x = f"{abs(linear_x):.6f}"
        abs_z = f"{abs(angular_z):.6f}"
        return f"1{sign_x}{abs_x}{sign_z}{abs_z}"

    def _send(self, frame: str):
        if not frame:
            return
        eol = self.get_parameter('eol').get_parameter_value().string_value
        try:
            self.ser.write((frame + eol).encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def cmd_vel_callback(self, msg: Twist):
        mode = self.get_parameter('protocol_mode').get_parameter_value().string_value
        if mode == 'signbit':
            frame = self._build_frame_signbit(msg.linear.x, msg.angular.z)
        else:
            frame = self._build_frame_offset100(msg.linear.x, msg.angular.z)
        self.last_cmd = frame
        self._send(frame)

    def heartbeat_send(self):
        if self.last_cmd:
            self._send(self.last_cmd)

    # -------- 수신: 파이프 뒤 첫째/둘째 값만 퍼블리시 --------
    def _reader_loop(self):
        buf = bytearray()
        scale = float(self.get_parameter('laser_scale').get_parameter_value().double_value)

        while self._rx_running:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.05)
                continue

            if chunk:
                buf.extend(chunk)

            # 개행 기준 라인 파싱
            while True:
                nl = buf.find(b'\n')
                if nl < 0:
                    break
                line = buf[:nl+1]
                del buf[:nl+1]

                try:
                    s = line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue
                if not s:
                    continue

                m = PAIR_REGEX.search(s)
                if not m:
                    # 필요한 경우: self.get_logger().debug(f"Unparsed: {s}")
                    continue

                try:
                    left_val  = float(m.group(1)) * scale
                    right_val = float(m.group(2)) * scale
                except ValueError:
                    continue

                msg_l = Float64(); msg_l.data = left_val
                msg_r = Float64(); msg_r.data = right_val
                self.pub_l.publish(msg_l)
                self.pub_r.publish(msg_r)
                # 필요한 경우: self.get_logger().info(f"L={left_val}, R={right_val}")

def main():
    rclpy.init()
    node = CmdVelToUart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._rx_running = False
        time.sleep(0.05)
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

