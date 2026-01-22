#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import FollowWaypoints


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')

        # === 사용자 설정 ===
        self.frame_id = "map"
        self.loop_forever = True           # ← 무한 반복 ON/OFF
        self.loop_delay_sec = 1.0          # ← 끝난 후 재시작까지 대기 시간(초)

        # Waypoint 좌표 (x, y, z)
        self.positions = [
            (0.0, 0.0, 0.0),
            (1.2, 0.0, 0.0),
            (2.4, 0.0, 0.0),
            (3.6, 0.0, 0.0),
        ]
        # 각 waypoint에서의 yaw(라디안)
        self.yaws = [
            math.radians(90),   # 북쪽
            math.radians(0),    # 동쪽
            math.radians(-90),  # 남쪽
            math.radians(180),  # 서쪽
        ]

        # Publishers
        self.path_pub = self.create_publisher(Path, "waypoint_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "waypoint_markers", 10)

        # Action client
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.goal_handle = None
        self._resend_timer = None  # ← 재전송용 원샷 타이머 핸들

        # 서버 대기 후 첫 전송
        self.get_logger().info("Waiting for follow_waypoints action server...")
        self.client.wait_for_server()
        self.get_logger().info("Server ready. Sending waypoints.")
        self.send_waypoints()

    def _build_goal_and_viz(self):
        """FollowWaypoints.Goal 과 시각화 메시지(Path/MarkerArray) 생성."""
        goal_msg = FollowWaypoints.Goal()

        path = Path()
        path.header.frame_id = self.frame_id

        marker_array = MarkerArray()

        for i, (pos, yaw) in enumerate(zip(self.positions, self.yaws)):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])

            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
            pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            goal_msg.poses.append(pose)
            path.poses.append(pose)

            # RViz 마커
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = pose.header.stamp
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        return goal_msg, path, marker_array

    def send_waypoints(self):
        """웨이포인트 목표 전송."""
        goal_msg, path, marker_array = self._build_goal_and_viz()

        # 시각화 갱신
        self.path_pub.publish(path)
        self.marker_pub.publish(marker_array)

        # 액션 전송
        self.get_logger().info("Sending FollowWaypoints goal...")
        future = self.client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """goal 수락/거절 결과 콜백."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("FollowWaypoints goal rejected.")
            self.goal_handle = None
            return
        self.goal_handle = goal_handle
        self.get_logger().info("FollowWaypoints goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        # 필요 시 현재 인덱스, 진행률 등 로깅 가능(메시지 필드는 Nav2 버전에 따라 다를 수 있음)
        pass

    def _on_result(self, future):
        """액션 완료 콜백: 성공/실패와 상관없이 필요 시 재전송."""
        try:
            result = future.result()
            # result.status / result.result 를 필요 시 확인 가능
            self.get_logger().info("Waypoints traversal finished.")
        except Exception as e:
            self.get_logger().warn(f"Waypoints result error: {e}")
        finally:
            self.goal_handle = None

        # 무한 반복이면 일정 시간 후 다시 전송
        if self.loop_forever:
            if self._resend_timer is None:
                self.get_logger().info(f"Looping enabled. Resend in {self.loop_delay_sec:.1f}s.")
                # 원샷 타이머: 콜백에서 바로 취소
                self._resend_timer = self.create_timer(self.loop_delay_sec, self._resend_once)

    def _resend_once(self):
        """원샷 재전송 타이머 콜백."""
        if self._resend_timer is not None:
            self._resend_timer.cancel()
            self._resend_timer = None
        self.send_waypoints()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
