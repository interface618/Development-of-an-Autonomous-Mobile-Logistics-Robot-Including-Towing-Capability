#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64, Bool, Int32
from ros2_aruco_interfaces.msg import ArucoMarkers
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from enum import Enum

class SearchState(Enum):
    IDLE = 0
    SEARCHING_LEFT = 1
    SEARCHING_RIGHT = 2

class SystemState(Enum):
    APPROACHING = 0
    LASER_CORRECTION = 1
    POST_WAIT = 2           # 레이저 보정 후 1초 대기
    ROTATE_180 = 3
    ROTATE_STOP_HOLD = 4    # ★ 회전 완료 후 1초 대기
    BACKOFF = 5             # 회전 후 짧은 후진(0.3s)
    DONE = 6

def ang_wrap_pi(a: float) -> float:
    while a > math.pi:  a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

class ArucoAlignerNode(Node):
    def __init__(self):
        super().__init__('aruco_aligner_node')

        # --- ⚙️ 튜닝 파라미터 ---
        self.ALIGN_IDS = [5, 4]      # 큰 마커(좌,우)
        self.FINAL_ID  = 6           # 최종(소형) 마커

        # 멀리서는 정렬 없이 직진만
        self.ALIGNMENT_START_DISTANCE = 1.0

        # 큰 마커 정렬 후 전환 거리(요청사항)
        self.HANDOVER_DISTANCE = 0.65

        # 최종 접근 종료 거리
        self.FINAL_DISTANCE = 0.22

        # 속도/게인
        self.LINEAR_SPEED = 0.11
        self.CREEP_SPEED_FACTOR = 0.3
        self.CENTER_GAIN = 0.5
        self.ANGLE_GAIN  = 0.3
        self.CENTER_ERROR_THRESHOLD = 0.03
        self.ANGLE_ERROR_THRESHOLD  = 0.05
        self.CAMERA_X_OFFSET = -0.01

        # 탐색
        self.SEARCH_ANGLE_LIMIT   = math.radians(60)
        self.SEARCH_ANGULAR_SPEED = 0.2

        # 레이저 보정
        self.LASER_ERROR_THRESHOLD = 0.003
        self.LASER_GAIN = 0.8
        self.MAX_LASER_CORRECTION_SPEED = 0.1
        self.MAX_VALID_LASER_DIST = 1.0

        # 보정 후 대기(정지) 시간
        self.POST_WAIT_SEC = 1.0     # 레이저 보정 후 1초 대기

        # 180도 회전 파라미터
        self.ROT_DIR = +1                      # +1: CCW, -1: CW
        self.ROT_DELTA = math.pi               # 180°
        self.ROT_KP = 1.7
        self.ROT_W_MIN = 0.10
        self.ROT_W_MAX = 0.80

        # ★ 회전 완료 후 정지 대기
        self.ROT_STOP_HOLD_SEC = 1.0          # 회전 후 1초 대기

        # 회전 후 짧은 후진
        self.BACKOFF_SEC = 5
        self.BACKOFF_SPEED = -0.12

        # 종료 대기(리미트 스위치 눌림 시 딜레이 후 종료)
        self.SHUTDOWN_DELAY_SEC = 6.0

        # --- 상태 변수 ---
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_image_msg = None
        self.align_marker_poses = {}
        self.final_marker_pose  = None
        self.last_twist_msg = Twist()
        self.camera_info_received = False

        self.left_laser_dist  = None
        self.right_laser_dist = None

        self.system_state = SystemState.APPROACHING
        self.final_approach_mode = False

        self.search_state = SearchState.IDLE
        self.search_start_yaw = None

        self.current_yaw = 0.0
        self.prev_yaw_raw = None
        self.yaw_unwrapped = None

        # 타이머/각도
        self.post_start_time = None
        self.rot_start_yaw   = None
        self.rot_target_yaw  = None
        self.rot_hold_start  = None
        self.back_start_time = None

        # 종료 플래그
        self.done_signal_sent = False
        self.limit_switch_pressed = False
        self.shutdown_initiated = False
        self.shutdown_timer_start = None

        # --- ROS 통신 설정 ---
        self.info_sub  = self.create_subscription(CameraInfo, "shark1/camera_info", self.camera_info_callback, 10)
        self.image_sub = self.create_subscription(Image, "shark1/image_raw", self.image_callback, 10)
        self.yaw_sub   = self.create_subscription(Float64, "/imu/yaw", self.yaw_callback, 10)

        self.align_marker_sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.align_marker_callback, 10)
        self.final_marker_sub = self.create_subscription(ArucoMarkers, '/aruco_4x4/aruco_markers', self.final_marker_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'shark1/cmd_vel', 10)

        # alignment 퍼블리셔 (활성)
        self.alignment_pub = self.create_publisher(Bool, 'shark1/alignment', 10)

        self.left_laser_sub  = self.create_subscription(Float64, 'shark1/distance_left',  self.left_laser_callback, 10)
        self.right_laser_sub = self.create_subscription(Float64, 'shark1/distance_right', self.right_laser_callback, 10)

        self.limit_switch_sub = self.create_subscription(Int32, 'shark1/limit_switch', self.limit_switch_callback, 10)

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.log_timer     = self.create_timer(1.0,  self.log_velocity_callback)

        self.get_logger().info("Aruco Aligner Node (Approach + Laser + 1s Wait + 180° Rotate + 1s Hold + Backoff) started.")

    # -------------------- 콜백 --------------------
    def limit_switch_callback(self, msg: Int32):
        if msg.data == 1 and not self.limit_switch_pressed:
            self.get_logger().info("Limit switch has been pressed (value=1).")
            self.limit_switch_pressed = True

    def left_laser_callback(self, msg: Float64):
        self.left_laser_dist = msg.data / 1000.0

    def right_laser_callback(self, msg: Float64):
        self.right_laser_dist = msg.data / 1000.0

    def yaw_callback(self, msg: Float64):
        deg_val = float(msg.data)
        raw = ang_wrap_pi(math.radians(deg_val))
        if self.prev_yaw_raw is None:
            self.prev_yaw_raw = raw
            self.yaw_unwrapped = raw
        else:
            d = raw - self.prev_yaw_raw
            if d > math.pi:      d -= 2.0 * math.pi
            elif d < -math.pi:   d += 2.0 * math.pi
            self.yaw_unwrapped += d
            self.prev_yaw_raw = raw
        self.current_yaw = raw

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs   = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera info received and locked.")

    def image_callback(self, msg):
        self.last_image_msg = msg

    def align_marker_callback(self, msg: ArucoMarkers):
        self.align_marker_poses.clear()
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id in self.ALIGN_IDS:
                self.align_marker_poses[marker_id] = msg.poses[i]

    def final_marker_callback(self, msg: ArucoMarkers):
        self.final_marker_pose = None
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id == self.FINAL_ID:
                self.final_marker_pose = msg.poses[i]
                break

    # -------------------- 메인 제어 루프 --------------------
    def control_loop(self):
        if not self.camera_info_received or self.last_image_msg is None:
            return

        state_text = "IDLE"
        laser_err_text = ""

        # DONE 상태
        if self.system_state == SystemState.DONE:
            if self.shutdown_initiated:
                dt = (self.get_clock().now() - self.shutdown_timer_start).nanoseconds * 1e-9
                state_text = f"SHUTTING DOWN in {self.SHUTDOWN_DELAY_SEC - dt:.1f}s"
                if dt >= self.SHUTDOWN_DELAY_SEC:
                    self.get_logger().info("Shutdown timer complete. Triggering KeyboardInterrupt to exit.")
                    raise KeyboardInterrupt()
            else:
                state_text = "SEQUENCE COMPLETE"
                self.publish_twist(0.0, 0.0)
                if not self.done_signal_sent:
                    msg = Bool(); msg.data = False
                    self.alignment_pub.publish(msg)
                    self.done_signal_sent = True
                    self.get_logger().info("Alignment/Rotation complete. Published True to /alignment.")
                if self.limit_switch_pressed:
                    self.get_logger().info(f"Limit switch active. Initiating {self.SHUTDOWN_DELAY_SEC}s shutdown.")
                    self.shutdown_timer_start = self.get_clock().now()
                    self.shutdown_initiated = True
            self.visualize_state(state_text, laser_err_text)
            return

        # 레이저 보정 후 1초 대기
        if self.system_state == SystemState.POST_WAIT:
            state_text = "WAIT AFTER LASER (1.0s)"
            self.publish_twist(0.0, 0.0)
            if self.post_start_time is not None:
                dt = (self.get_clock().now() - self.post_start_time).nanoseconds * 1e-9
                if dt >= self.POST_WAIT_SEC:
                    if self.yaw_unwrapped is None:
                        self.get_logger().warn("Yaw not available, finish without rotation.")
                        self.system_state = SystemState.DONE
                    else:
                        self.system_state = SystemState.ROTATE_180
                        self.rot_start_yaw  = self.yaw_unwrapped
                        self.rot_target_yaw = self.rot_start_yaw + self.ROT_DIR * abs(self.ROT_DELTA)
                        self.get_logger().info(
                            f"Wait done → Start rotation: start={math.degrees(self.rot_start_yaw):.2f}°, "
                            f"target={math.degrees(self.rot_target_yaw):.2f}°, dir={self.ROT_DIR}"
                        )
            self.visualize_state(state_text, laser_err_text)
            return

        # 180° 회전
        if self.system_state == SystemState.ROTATE_180:
            state_text = "ROTATING 180°"
            if self.yaw_unwrapped is None or self.rot_target_yaw is None:
                self.publish_twist(0.0, 0.0)
                self.visualize_state(state_text, laser_err_text)
                return

            reached = (self.ROT_DIR > 0 and self.yaw_unwrapped >= self.rot_target_yaw) or \
                      (self.ROT_DIR < 0 and self.yaw_unwrapped <= self.rot_target_yaw)
            if reached:
                # 회전 완료 → 1초 정지 대기
                self.system_state = SystemState.ROTATE_STOP_HOLD
                self.rot_hold_start = self.get_clock().now()
                self.publish_twist(0.0, 0.0)
                self.get_logger().info("Rotation target reached → hold 1.0s")
                self.visualize_state(state_text, laser_err_text)
                return

            err = self.rot_target_yaw - self.yaw_unwrapped
            w = self.ROT_KP * err
            if abs(w) > 1e-6:
                w = max(self.ROT_W_MIN, min(abs(w), self.ROT_W_MAX)) * (1.0 if w > 0 else -1.0)
            else:
                w = 0.0
            self.publish_twist(0.0, w)
            self.visualize_state(state_text, laser_err_text)
            return

        # ★ 회전 후 1초 정지 대기
        if self.system_state == SystemState.ROTATE_STOP_HOLD:
            state_text = "ROTATE HOLD (1.0s)"
            self.publish_twist(0.0, 0.0)
            if self.rot_hold_start is not None:
                dt = (self.get_clock().now() - self.rot_hold_start).nanoseconds * 1e-9
                if dt >= self.ROT_STOP_HOLD_SEC:
                    self.system_state = SystemState.BACKOFF
                    self.back_start_time = self.get_clock().now()
            self.visualize_state(state_text, laser_err_text)
            return

        # 회전 직후 짧은 후진
        if self.system_state == SystemState.BACKOFF:
            state_text = "BACKING OFF (short)"
            if self.back_start_time is not None:
                dt = (self.get_clock().now() - self.back_start_time).nanoseconds * 1e-9
                if dt < self.BACKOFF_SEC:
                    self.publish_twist(self.BACKOFF_SPEED, 0.0)
                else:
                    self.system_state = SystemState.DONE
                    self.publish_twist(0.0, 0.0)
            self.visualize_state(state_text, laser_err_text)
            return

        # 레이저 보정
        if self.system_state == SystemState.LASER_CORRECTION:
            state_text = "PERFORMING LASER CORRECTION"
            is_left_valid  = self.left_laser_dist  is not None and 0 < self.left_laser_dist  < self.MAX_VALID_LASER_DIST
            is_right_valid = self.right_laser_dist is not None and 0 < self.right_laser_dist < self.MAX_VALID_LASER_DIST
            if is_left_valid and is_right_valid:
                laser_error = self.right_laser_dist - self.left_laser_dist
                laser_err_text = f"Laser Err: {laser_error:.4f} m"
                self.get_logger().info(
                    f"Correction - L: {self.left_laser_dist*1000:.1f} mm, "
                    f"R: {self.right_laser_dist*1000:.1f} mm, Err: {laser_error:.4f} m",
                    throttle_duration_sec=0.5
                )
                if abs(laser_error) < self.LASER_ERROR_THRESHOLD:
                    # 보정 완료 → 1초 대기 상태로 전환
                    self.system_state = SystemState.POST_WAIT
                    self.post_start_time = self.get_clock().now()
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().info("Laser correction complete → waiting 1.0s before rotation.")
                else:
                    angular_vel = self.LASER_GAIN * laser_error
                    angular_vel = float(np.clip(angular_vel, -self.MAX_LASER_CORRECTION_SPEED, self.MAX_LASER_CORRECTION_SPEED))
                    self.publish_twist(0.0, angular_vel)
            else:
                laser_err_text = "Waiting for valid laser data..."
                self.publish_twist(0.0, 0.0)
            self.visualize_state(state_text, laser_err_text)
            return

        # 탐색(서치)
        pose_left  = self.align_marker_poses.get(self.ALIGN_IDS[0])
        pose_right = self.align_marker_poses.get(self.ALIGN_IDS[1])

        if self.search_state != SearchState.IDLE:
            markers_found = (not self.final_approach_mode and pose_left and pose_right) or \
                            (self.final_approach_mode and self.final_marker_pose)
            if markers_found:
                self.get_logger().info("Markers found, stopping search.")
                self.search_state = SearchState.IDLE
                self.search_start_yaw = None
            else:
                if self.search_start_yaw is None:
                    self.search_start_yaw = self.current_yaw
                yaw_diff = self.normalize_angle(self.current_yaw - self.search_start_yaw)

                if self.search_state == SearchState.SEARCHING_LEFT:
                    state_text = f"SEARCHING LEFT ({math.degrees(yaw_diff):.1f}°)"
                    if yaw_diff < self.SEARCH_ANGLE_LIMIT:
                        self.publish_twist(0.0, self.SEARCH_ANGULAR_SPEED)
                    else:
                        self.search_state = SearchState.SEARCHING_RIGHT
                elif self.search_state == SearchState.SEARCHING_RIGHT:
                    state_text = f"SEARCHING RIGHT ({math.degrees(yaw_diff):.1f}°)"
                    if yaw_diff > -self.SEARCH_ANGLE_LIMIT:
                        self.publish_twist(0.0, -self.SEARCH_ANGULAR_SPEED)
                    else:
                        self.search_state = SearchState.IDLE
                        state_text = "SEARCH FAILED, WAITING"
                self.visualize_state(state_text, laser_err_text)
                return

        # 접근/정렬
        if not self.final_approach_mode:
            if pose_left and pose_right:
                dist_z_avg = (pose_left.position.z + pose_right.position.z) / 2.0

                if dist_z_avg > self.ALIGNMENT_START_DISTANCE:
                    state_text = f"BLIND APPROACH | Dist: {dist_z_avg:.2f}m"
                    vx = self.LINEAR_SPEED * self.CREEP_SPEED_FACTOR
                    self.publish_twist(vx, 0.0)
                else:
                    if dist_z_avg <= self.HANDOVER_DISTANCE:
                        self.final_approach_mode = True
                        state_text = "SWITCHING... PAUSE"
                        self.publish_twist(0.0, 0.0)
                    else:
                        state_text = f"ALIGNING (Large Markers) | Dist: {dist_z_avg:.2f}m"
                        markers_center_x = (pose_left.position.x + pose_right.position.x) / 2.0
                        err_center_x = markers_center_x - self.CAMERA_X_OFFSET
                        err_angle_z = pose_left.position.z - pose_right.position.z

                        angular_vel = -(self.CENTER_GAIN * err_center_x) - (self.ANGLE_GAIN * err_angle_z)

                        is_camera_aligned = (abs(err_center_x) < self.CENTER_ERROR_THRESHOLD) and \
                                            (abs(err_angle_z)  < self.ANGLE_ERROR_THRESHOLD)

                        vx = self.LINEAR_SPEED if is_camera_aligned else self.LINEAR_SPEED * self.CREEP_SPEED_FACTOR
                        self.publish_twist(vx, angular_vel)
            else:
                state_text = "LOST LARGE MARKERS, STARTING SEARCH..."
                self.search_state = SearchState.SEARCHING_LEFT
                self.search_start_yaw = self.current_yaw
                self.publish_twist(0.0, 0.0)

        else:
            # 최종(소형) 마커 기반 최종 접근
            if self.final_marker_pose:
                state_text = "FINAL APPROACH (Small Marker)"
                dist_final = self.final_marker_pose.position.z
                if dist_final > self.FINAL_DISTANCE:
                    err_final_x = self.final_marker_pose.position.x - self.CAMERA_X_OFFSET
                    is_camera_aligned = abs(err_final_x) < self.CENTER_ERROR_THRESHOLD
                    vx = self.LINEAR_SPEED * 0.5 if is_camera_aligned else 0.0
                    wz = -self.CENTER_GAIN * err_final_x
                    self.publish_twist(vx, wz)
                else:
                    self.get_logger().info("Target distance reached. Starting final laser correction.")
                    self.system_state = SystemState.LASER_CORRECTION
                    self.publish_twist(0.0, 0.0)
            else:
                state_text = "LOST FINAL MARKER, STARTING SEARCH..."
                self.search_state = SearchState.SEARCHING_LEFT
                self.search_start_yaw = self.current_yaw
                self.publish_twist(0.0, 0.0)

        self.visualize_state(state_text, laser_err_text)

    # -------------------- 유틸 --------------------
    def normalize_angle(self, angle):
        return ang_wrap_pi(angle)

    def publish_twist(self, vx: float, wz: float):
        if rclpy.ok():
            msg = Twist()
            msg.linear.x  = float(vx)
            msg.angular.z = float(wz)
            self.cmd_vel_pub.publish(msg)
            self.last_twist_msg = msg

    def log_velocity_callback(self):
        if rclpy.ok():
            self.get_logger().info(
                f"State: {self.system_state.name} | Search: {self.search_state.name} | "
                f"Linear: {self.last_twist_msg.linear.x:.3f} m/s, "
                f"Angular: {self.last_twist_msg.angular.z:.3f} rad/s"
            )

    def visualize_state(self, state_text, laser_text):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image_msg, 'bgr8')
            cv2.putText(cv_image, state_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, laser_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            all_poses = list(self.align_marker_poses.values())
            if self.final_marker_pose:
                all_poses.append(self.final_marker_pose)

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                for pose in all_poses:
                    pos = pose.position
                    ori = pose.orientation
                    tvec = np.array([pos.x, pos.y, pos.z], dtype=np.float32)
                    r_mat = np.array([
                        [1 - 2*ori.y**2 - 2*ori.z**2, 2*ori.x*ori.y - 2*ori.z*ori.w, 2*ori.x*ori.z + 2*ori.y*ori.w],
                        [2*ori.x*ori.y + 2*ori.z*ori.w, 1 - 2*ori.x**2 - 2*ori.z**2, 2*ori.y*ori.z - 2*ori.x*ori.w],
                        [2*ori.x*ori.z - 2*ori.y*ori.w, 2*ori.y*ori.z + 2*ori.x*ori.w, 1 - 2*ori.x**2 - 2*ori.y**2]
                    ], dtype=np.float32)
                    rvec, _ = cv2.Rodrigues(r_mat)
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

            cv2.imshow("Aruco Aligner View", cv_image)
            cv2.waitKey(1)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ArucoAlignerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Shutdown requested. Stopping robot and destroying node...')
            node.publish_twist(0.0, 0.0)
            node.destroy_node()
    finally:
        cv2.destroyAllWindows()
        print("\nProcess finished cleanly.")

if __name__ == '__main__':
    main()

