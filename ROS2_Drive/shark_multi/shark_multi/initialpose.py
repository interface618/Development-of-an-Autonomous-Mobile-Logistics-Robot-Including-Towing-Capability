#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

TOPIC = "/shark1/initialpose"
FRAME_ID = "map"
INIT_X = 2.0
INIT_Y = 0.0
INIT_YAW_DEG = 90.0
DELAY_SEC = 10.0
REPEAT_ONCE_AFTER_SEC = 0.5   # 한 번 더 보내기 원치 않으면 0.0

def yaw_to_quat(yaw_rad: float) -> Quaternion:
    half = yaw_rad * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

class InitialPoseOnce(Node):
    def __init__(self):
        super().__init__("initialpose")

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, TOPIC, qos)

        self._sent_first = False
        self._timer_first = self.create_timer(DELAY_SEC, self._publish_once)
        self._timer_second = None  # 둘째 타이머 핸들

        self.get_logger().info(
            f"Will publish initialpose to '{TOPIC}' in {DELAY_SEC:.1f}s "
            f"(frame_id='{FRAME_ID}', x={INIT_X:.3f}, y={INIT_Y:.3f}, yaw={INIT_YAW_DEG:.1f}°)"
        )

    def _make_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(INIT_X)
        msg.pose.pose.position.y = float(INIT_Y)
        msg.pose.pose.position.z = 0.0
        yaw = math.radians(float(INIT_YAW_DEG))
        msg.pose.pose.orientation = yaw_to_quat(yaw)

        cov = [0.0] * 36
        cov[0] = (0.05 ** 2)                       # x
        cov[7] = (0.05 ** 2)                       # y
        cov[35] = (math.radians(5.0) ** 2)         # yaw
        msg.pose.covariance = cov
        return msg

    def _publish_once(self):
        if self._sent_first:
            return
        self.pub.publish(self._make_msg())
        self.get_logger().info("Published initialpose (1/1 or 1/2).")
        self._sent_first = True

        # 이 첫 번째 타이머는 더 이상 필요 없음 → 취소
        if self._timer_first is not None:
            self._timer_first.cancel()
            self._timer_first = None

        # 두 번째 발행을 원하면 원샷 타이머를 새로 만들어서, 콜백에서 즉시 취소
        if REPEAT_ONCE_AFTER_SEC > 0.0:
            self._timer_second = self.create_timer(REPEAT_ONCE_AFTER_SEC, self._publish_second)

    def _publish_second(self):
        self.pub.publish(self._make_msg())
        self.get_logger().info("Published initialpose (2/2).")
        if self._timer_second is not None:
            self._timer_second.cancel()
            self._timer_second = None

def main():
    rclpy.init()
    node = InitialPoseOnce()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
