#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

# ===== 고정 설정 =====
SCAN_IN  = '/shark1/scan'        # 원본 입력
SCAN_OUT = '/shark1/scan_front'   # 결과 출력 (옵션 B: '/shark1/scan' 로 덮어쓰기)
REAR_THRESHOLD_RAD = math.pi / 2.0   # 90도
FILL_INF = float('inf')              # 제외 구간 채움값
# =====================

def wrap_pi(a: float) -> float:
    """각도를 [-pi, pi]로 래핑"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class RearHalfScan(Node):
    def __init__(self):
        super().__init__('rear_half_scan')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(LaserScan, SCAN_OUT, qos)
        self.sub = self.create_subscription(LaserScan, SCAN_IN, self.cb, qos)

        self.get_logger().info(
            f'RearHalfScan: {SCAN_IN} → {SCAN_OUT} (후방만 유효, 전방은 INF로 마스킹)'
        )

    def cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment <= 0.0:
            return

        amin = msg.angle_min
        ainc = msg.angle_increment

        # 1) 각 인덱스의 각도를 래핑해 후방(|θ| >= 90°) 여부를 판단
        include = [False] * n
        for i in range(n):
            ang = amin + i * ainc       # 메시지의 실제 각도 (래핑 전)
            ang_wrapped = wrap_pi(ang)  # [-pi, pi]
            include[i] = (abs(ang_wrapped) >= REAR_THRESHOLD_RAD)

        # 2) 유효 구간의 처음/끝 인덱스 찾기
        try:
            first_idx = next(i for i, ok in enumerate(include) if ok)
            last_idx  = n - 1 - next(i for i, ok in enumerate(reversed(include)) if ok)
        except StopIteration:
            # 전부 제외된 경우
            return

        # 3) 연속 각도 범위를 first_idx..last_idx로 잡고,
        #    후방이 아닌 인덱스는 range를 INF로 채워 단일 LaserScan으로 발행
        out_len = last_idx - first_idx + 1
        out_ranges = [0.0] * out_len
        out_intens = [0.0] * out_len if msg.intensities else []

        for k in range(out_len):
            i = first_idx + k
            if include[i]:
                out_ranges[k] = msg.ranges[i]
                if msg.intensities:
                    out_intens[k] = msg.intensities[i]
            else:
                out_ranges[k] = FILL_INF
                if msg.intensities:
                    out_intens[k] = 0.0

        # 4) 메시지 구성 (각도 그리드는 원본과 동일한 증분을 유지)
        out = LaserScan()
        out.header = msg.header
        out.angle_min = amin + ainc * first_idx
        out.angle_max = amin + ainc * last_idx
        out.angle_increment = ainc
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = out_ranges
        out.intensities = out_intens if msg.intensities else []
        self.pub.publish(out)

def main():
    rclpy.init()
    node = RearHalfScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
