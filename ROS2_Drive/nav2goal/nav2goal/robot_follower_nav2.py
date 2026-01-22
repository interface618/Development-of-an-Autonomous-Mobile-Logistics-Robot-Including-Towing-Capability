#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

# ---------- 너가 준 주차(오프셋) 함수: cm/deg ----------
def parking_position(x_t, y_t, theta_t_deg, distance_cm=10):
    """
    정차된 로봇 뒤에 평행하게 주차할 위치 계산
    x_t, y_t : 정차된 로봇의 중심 좌표 (cm 단위)
    theta_t_deg : 정차된 로봇의 방향 (도 단위, 0도 = x축 방향)
    distance_cm : 뒤로 떨어질 거리 (기본값 10cm)
    """
    theta_t = math.radians(theta_t_deg)
    forward = (math.cos(theta_t), math.sin(theta_t))
    backward = (-forward[0], -forward[1])
    x_p = x_t + backward[0] * distance_cm
    y_p = y_t + backward[1] * distance_cm
    theta_p_deg = theta_t_deg  # 평행(같은 방향)
    return x_p, y_p, theta_p_deg
# -----------------------------------------------------

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

def quaternion_to_yaw(q: Quaternion) -> float:
    # yaw from quaternion (Z-Y-X)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_normalize(a: float) -> float:
    while a > math.pi:  a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

class RobotFollower(Node):
    def __init__(self):
        super().__init__('robot_follower')

        # Parameters (모두 명시적 기본값)
        self.declare_parameter('distance', 0.1)                 # m (10cm)
        self.declare_parameter('follow_side', 'behind')         # 'behind' or 'front' (주차 함수는 'behind' 가정)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pos_reissue_threshold', 0.10)   # m
        self.declare_parameter('yaw_reissue_threshold_deg', 5.0)# deg

        # Optional one-shot initial target
        self.declare_parameter('use_initial_goal', False)
        self.declare_parameter('initial_x', 0.0)                # m
        self.declare_parameter('initial_y', 0.0)                # m
        self.declare_parameter('initial_theta_deg', 0.0)        # deg

        # Read params
        self.distance    = float(self.get_parameter('distance').value)             # m
        self.follow_side = str(self.get_parameter('follow_side').value).lower()
        self.frame_id    = str(self.get_parameter('frame_id').value)
        self.pos_thr     = float(self.get_parameter('pos_reissue_threshold').value)
        self.yaw_thr     = math.radians(float(self.get_parameter('yaw_reissue_threshold_deg').value))

        use_initial  = bool(self.get_parameter('use_initial_goal').value)
        init_x       = float(self.get_parameter('initial_x').value)       # m
        init_y       = float(self.get_parameter('initial_y').value)       # m
        init_th_deg  = float(self.get_parameter('initial_theta_deg').value) # deg

        # Publishers / Subscribers / Action
        self.marker_pub = self.create_publisher(MarkerArray, '/follower_goal_markers', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/target_pose', self.on_target_pose, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.server_ready = False
        self.create_timer(0.5, self._check_server)

        # State
        self.current_goal_handle = None
        self.last_sent_goal: Optional[PoseStamped] = None

        # one-shot initial (init_x,y,theta_deg는 "타깃 로봇 포즈"라고 가정)
        if use_initial:
            self.get_logger().info(
                f'Using initial target (x={init_x:.3f} m, y={init_y:.3f} m, theta={init_th_deg:.1f}°)'
            )
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position = Point(x=init_x, y=init_y, z=0.0)
            yaw = math.radians(init_th_deg)
            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
            ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            # 서버 준비되면 전송
            self.create_timer(0.2, lambda: self._maybe_send_follow_goal(ps))

    def _check_server(self):
        if not self.server_ready and self.nav_client.wait_for_server(timeout_sec=0.0):
            self.server_ready = True
            self.get_logger().info('Nav2 navigate_to_pose action server is ready.')

    def on_target_pose(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"Target frame_id '{msg.header.frame_id}' != follower frame_id '{self.frame_id}'. TF 변환 필요."
            )
        self._maybe_send_follow_goal(msg)

    # ---- cm/deg 함수를 ROS(m/rad)에서 쓰기 위한 래퍼 ----
    def _compute_goal_from_target(self, tx_m: float, ty_m: float, tyaw_rad: float):
        """
        입력: m, rad  / 출력: gx_m, gy_m, gyaw_rad
        parking_position()은 cm/deg 기반이므로 단위 변환해 호출
        follow_side == 'front'면 앞쪽 +distance, behind면 뒤쪽 -distance
        """
        # 타깃 각 -> deg
        theta_t_deg = math.degrees(tyaw_rad)

        # 거리(m) -> cm. behind는 뒤로(+), front는 앞으로(+)
        d_cm = abs(self.distance) * 100.0
        if self.follow_side == 'front':
            # 앞쪽으로 가고 싶다면 “뒤로 떨어지는 함수”를 반대로 쓰기 위해
            # 타깃 각에 +180°를 더해 뒤쪽을 앞쪽으로 치환
            theta_for_front_deg = angle_normalize(math.radians(theta_t_deg + 180.0))
            theta_for_front_deg = math.degrees(theta_for_front_deg)
            x_t_cm = tx_m * 100.0
            y_t_cm = ty_m * 100.0
            gx_cm, gy_cm, gyaw_deg = parking_position(x_t_cm, y_t_cm, theta_for_front_deg, d_cm)
            # 주의: 위 변환에서 yaw는 이미 타깃 yaw(평행)과 같게 됨
        else:
            # behind: 함수 그대로 사용
            x_t_cm = tx_m * 100.0
            y_t_cm = ty_m * 100.0
            gx_cm, gy_cm, gyaw_deg = parking_position(x_t_cm, y_t_cm, theta_t_deg, d_cm)

        gx_m = gx_cm / 100.0
        gy_m = gy_cm / 100.0
        gyaw_rad = math.radians(gyaw_deg)  # 평행(같은 yaw)
        return gx_m, gy_m, gyaw_rad
    # ---------------------------------------------------

    def _maybe_send_follow_goal(self, target: PoseStamped):
        if not self.server_ready:
            self.get_logger().warn('navigate_to_pose 서버 대기 중입니다. 준비되면 goal을 전송합니다.')
            return

        # 1) 타깃 포즈 (m, rad)
        tx = target.pose.position.x
        ty = target.pose.position.y
        tyaw = quaternion_to_yaw(target.pose.orientation)

        # 2) 너의 parking_position 공식을 이용한 오프셋 + 평행 yaw 계산
        gx, gy, gyaw = self._compute_goal_from_target(tx, ty, tyaw)

        # Compose PoseStamped goal
        goal_ps = PoseStamped()
        goal_ps.header.frame_id = self.frame_id
        goal_ps.header.stamp = self.get_clock().now().to_msg()
        goal_ps.pose.position = Point(x=gx, y=gy, z=0.0)
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, gyaw)
        goal_ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 로깅(확인용)
        self.get_logger().info(
            f"[FOLLOW] target=({tx:.3f}, {ty:.3f}, {math.degrees(tyaw):.1f}°) "
            f"-> goal=({gx:.3f}, {gy:.3f}, {math.degrees(gyaw):.1f}°), "
            f"d={self.distance:.3f}m, side={self.follow_side}"
        )

        # 3) 재발행 판단(히스테리시스)
        if self.last_sent_goal is not None:
            dx = goal_ps.pose.position.x - self.last_sent_goal.pose.position.x
            dy = goal_ps.pose.position.y - self.last_sent_goal.pose.position.y
            dpos = math.hypot(dx, dy)
            last_yaw = quaternion_to_yaw(self.last_sent_goal.pose.orientation)
            dyaw = angle_normalize(gyaw - last_yaw)
            if dpos < self.pos_thr and abs(dyaw) < self.yaw_thr:
                return
            if self.current_goal_handle is not None:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass

        # 4) 전송
        self._publish_marker(goal_ps)
        self._send_nav_goal(goal_ps)
        self.last_sent_goal = goal_ps

    def _send_nav_goal(self, goal_pose: PoseStamped):
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected.')
            self.current_goal_handle = None
            return
        self.current_goal_handle = goal_handle
        self.get_logger().info('NavigateToPose goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        pass

    def _on_result(self, future):
        try:
            _ = future.result().result
            self.get_logger().info('NavigateToPose goal finished.')
        except Exception as e:
            self.get_logger().warn(f'NavigateToPose result error: {e}')
        finally:
            self.current_goal_handle = None

    def _publish_marker(self, goal_ps: PoseStamped):
        arr = MarkerArray()
        m = Marker()
        m.header = goal_ps.header
        m.ns = 'follower_goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = goal_ps.pose
        m.scale.x = 0.5
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        arr.markers.append(m)
        self.marker_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = RobotFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
