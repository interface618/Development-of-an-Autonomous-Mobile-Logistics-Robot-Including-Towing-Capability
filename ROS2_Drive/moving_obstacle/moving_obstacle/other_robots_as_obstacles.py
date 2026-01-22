#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

# XYZ32 포인트클라우드 생성 유틸
from sensor_msgs_py import point_cloud2


# ==========================
# 고정 설정(원하시는 값으로 수정)
# ==========================
# 퍼블리시할 토픽 이름 (Nav2 YAML의 obstacle_layer/others.topic 과 동일해야 함)
OUTPUT_TOPIC: str = "/shark1/other_robots_cloud"

# 포인트클라우드의 frame_id (보통 'map' 권장. local_costmap.global_frame 로 TF 변환 가능해야 함)
OUTPUT_FRAME: str = "shark1/odom"

# 장애물로 표시할 '상대 로봇'들의 base_link 프레임들
# 예) shark1에서 실행 → shark0을 장애물로 찍고 싶으면 아래처럼
TARGET_BASE_FRAMES: List[str] = [
    "shark0/base_footprint",
    # 필요 시 여러 대 추가
    # "shark2/base_link",
]

# 상대 로봇의 반경 + 여유 (m)
ROBOT_RADIUS: float = 0.4      # 상대 로봇의 robot_radius
EXTRA_MARGIN: float = 0.05      # 여유 (조금 크게 찍고 싶으면 늘리기)

# 원 형태로 찍을 포인트 개수 (24~36 권장)
POINTS_PER_ROBOT: int = 24

# 퍼블리시 주기(Hz)
PUBLISH_RATE_HZ: float = 10.0
# ==========================


class OtherRobotsAsObstacles(Node):
    def __init__(self):
        super().__init__("other_robots_as_obstacles")

        if not TARGET_BASE_FRAMES:
            self.get_logger().warn(
                "TARGET_BASE_FRAMES가 비어 있습니다. 장애물로 찍을 대상이 없습니다."
            )
        else:
            self.get_logger().info(f"Targets: {TARGET_BASE_FRAMES}")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(PointCloud2, OUTPUT_TOPIC, 10)

        # Timer
        period = 1.0 / max(1e-3, PUBLISH_RATE_HZ)
        self.create_timer(period, self._tick)

    def _tick(self):
        pts = self._collect_points()
        if not pts:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = OUTPUT_FRAME

        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub.publish(cloud)

    def _collect_points(self) -> List[Tuple[float, float, float]]:
        """TARGET_BASE_FRAMES의 현재 위치를 OUTPUT_FRAME 기준으로 TF 조회하고,
        각 위치 주변을 원형 포인트로 샘플링하여 반환.
        """
        points: List[Tuple[float, float, float]] = []

        radius = ROBOT_RADIUS + EXTRA_MARGIN
        N = max(6, int(POINTS_PER_ROBOT))

        for base in TARGET_BASE_FRAMES:
            try:
                tf = self.tf_buffer.lookup_transform(
                    OUTPUT_FRAME,   # target frame (우리가 출력할 좌표계)
                    base,           # source frame (상대 로봇의 base_link)
                    rclpy.time.Time()  # 최신
                )
            except Exception as e:
                # 디버그가 필요하면 warn/info로 올리세요
                self.get_logger().debug(f"TF lookup failed for {base}: {e}")
                continue

            cx = tf.transform.translation.x
            cy = tf.transform.translation.y
            cz = tf.transform.translation.z  # 보통 0

            # 중심점(선택: 코스트맵에 더 확실히 찍히도록)
            points.append((cx, cy, cz))

            # 둘레 포인트들
            for k in range(N):
                ang = 2.0 * math.pi * k / N
                px = cx + radius * math.cos(ang)
                py = cy + radius * math.sin(ang)
                pz = cz
                points.append((px, py, pz))

        return points


def main(args=None):
    rclpy.init(args=args)
    node = OtherRobotsAsObstacles()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
