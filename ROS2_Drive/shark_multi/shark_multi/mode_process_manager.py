#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import signal
import subprocess
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped


# ----------------- 강력 종료 유틸 (그룹 + 자식 트리) -----------------
def _list_children_pids(ppid: int) -> list[int]:
    """직계 자식 PID 목록 (ps 사용)"""
    try:
        out = subprocess.check_output(['ps', '-o', 'pid=', '--ppid', str(ppid)], text=True)
        return [int(x.strip()) for x in out.splitlines() if x.strip().isdigit()]
    except Exception:
        return []

def _kill_pid(pid: int, sig: int):
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass

def _kill_process_tree(root_pid: int, sig: int):
    """ppid 트리 기준으로 자식들 먼저, 마지막에 루트에 시그널"""
    for cpid in _list_children_pids(root_pid):
        _kill_pid(cpid, sig)
    _kill_pid(root_pid, sig)

def terminate_proc(p: Optional[subprocess.Popen], name: str, logger) -> None:
    """
    ros2 launch 트리 완전 종료:
    1) SIGINT (그룹 + PPID 트리) → 대기
    2) SIGTERM (그룹 + PPID 트리) → 대기
    3) SIGKILL (그룹 + PPID 트리)
    """
    if p is None or p.poll() is not None:
        return

    try:
        pgid = os.getpgid(p.pid)  # 프로세스 그룹 ID
    except Exception:
        pgid = None

    def alive() -> bool:
        return p.poll() is None

    def send(sig: int, tag: str):
        # 그룹 전체
        if pgid:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
        else:
            try:
                p.send_signal(sig)
            except ProcessLookupError:
                pass
        # PPID 트리(자식들 포함)
        _kill_process_tree(p.pid, sig)
        logger.info(f'[{name}] sent {tag} to group/tree')

    def wait_for_exit(seconds: float) -> bool:
        t0 = time.time()
        while alive() and (time.time() - t0) < seconds:
            time.sleep(0.1)
        return not alive()

    logger.info(f'[{name}] stopping … (SIGINT)')
    send(signal.SIGINT, 'SIGINT')
    if wait_for_exit(8.0):
        return

    logger.warn(f'[{name}] still running. Sending SIGTERM…')
    send(signal.SIGTERM, 'SIGTERM')
    if wait_for_exit(5.0):
        return

    logger.warn(f'[{name}] did not exit. Forcing SIGKILL…')
    send(signal.SIGKILL, 'SIGKILL')
    wait_for_exit(3.0)


# ----------------- 런치 실행 유틸 -----------------
def popen_launch(pkg: str, launch_file: str, args: List[str]) -> subprocess.Popen:
    """
    예) ros2 launch shark_multi dr_bringup_launch.py params_file:=... map:=...
    - 부모 터미널로 로그 그대로 출력
    - 별도 프로세스 그룹(start_new_session=True)로 띄워 그룹 종료 가능
    """
    cmd = ['ros2', 'launch', pkg, launch_file] + args
    print('[spawn]', ' '.join(cmd))
    return subprocess.Popen(
        cmd,
        stdout=None,
        stderr=None,
        env=os.environ.copy(),
        start_new_session=True
    )


# ----------------- 메인 노드 -----------------
class ModeProcessManager(Node):
    """
    /limit_switch (std_msgs/Int32)
      0 → Driving 세트 실행 (Traction 세트 종료)
      1 → Traction 세트 실행 (Driving 세트 종료)

    * 오직 두 런치(브링업/로컬)만 실행/종료
    * 전환 후 /initialpose 로 최근 /amcl_pose 재주입(구독자 대기 포함)
    * 네임스페이스 자동 주입/노드대기/서비스대기 없음
    """

    def __init__(self):
        super().__init__('mode_process_manager')

        # ===== 최소 파라미터 =====
        self.declare_parameter('limit_switch_topic', '/shark1/limit_switch')
        self.declare_parameter('debounce_ms', 800)
        self.declare_parameter('start_mode', 0)                 # 0=Driving, 1=Traction
        self.declare_parameter('wait_after_bringup_sec', 1.0)   # bringup → localization 유예
        self.declare_parameter('wait_before_initialpose_sec', 1.0)

        # ===== 현재 위치 유지(AMCL 재주입) =====
        self.declare_parameter('amcl_pose_topic', '/shark1/amcl_pose')     # 캐시 입력
        self.declare_parameter('initialpose_topic', '/shark1/initialpose') # 퍼블리시 대상
        self.declare_parameter('map_frame_id', 'map')                      # initialpose 헤더 프레임
        self.declare_parameter('initialpose_repeat', 10)                   # 1회 호출당 발행 횟수
        self.declare_parameter('initialpose_interval_sec', 0.1)            # 발행 간격
        # (옵션) 더 안정적으로: 구독자 대기/퍼블리시 윈도우
        self.declare_parameter('initialpose_wait_subscriber_sec', 10.0)    # 최대 구독자 대기
        self.declare_parameter('initialpose_publish_window_sec',  3.0)     # 대기 후 계속 퍼블리시 시간

        # ===== 런치 세트(네가 쓰던 그대로) =====
        # Driving
        self.declare_parameter('dr_bringup_pkg',  'shark_multi')
        self.declare_parameter('dr_bringup_file', 'dr_bringup_launch.py')
        self.declare_parameter('dr_bringup_args', [
            'params_file:=/home/hiru/shark_ws/src/shark_multi/config/Driving_mode.yaml'
        ])
        self.declare_parameter('dr_local_pkg',  'shark_multi')
        self.declare_parameter('dr_local_file', 'dr_localization_launch.py')
        self.declare_parameter('dr_local_args', [
            'map:=/home/hiru/shark_ws/src/shark_multi/map/center.yaml',
            'params_file:=/home/hiru/shark_ws/src/shark_multi/config/Driving_mode.yaml'
        ])

        # Traction
        self.declare_parameter('tr_bringup_pkg',  'shark_multi')
        self.declare_parameter('tr_bringup_file', 'tr_bringup_launch.py')
        self.declare_parameter('tr_bringup_args', [
            'params_file:=/home/hiru/shark_ws/src/shark_multi/config/Traction_mode.yaml'
        ])
        self.declare_parameter('tr_local_pkg',  'shark_multi')
        self.declare_parameter('tr_local_file', 'tr_localization_launch.py')
        self.declare_parameter('tr_local_args', [
            'map:=/home/hiru/shark_ws/src/shark_multi/map/center.yaml',
            'params_file:=/home/hiru/shark_ws/src/shark_multi/config/Traction_mode.yaml'
        ])

        # ===== 로딩 =====
        self.topic_limit = self.get_parameter('limit_switch_topic').value
        self.debounce_ms = int(self.get_parameter('debounce_ms').value)
        self.wait_after_bringup = float(self.get_parameter('wait_after_bringup_sec').value)
        self.wait_before_initialpose = float(self.get_parameter('wait_before_initialpose_sec').value)

        # AMCL 재주입 관련
        self.topic_amcl  = self.get_parameter('amcl_pose_topic').value
        self.topic_init  = self.get_parameter('initialpose_topic').value
        self.map_frame   = self.get_parameter('map_frame_id').value
        self.initialpose_repeat = int(self.get_parameter('initialpose_repeat').value)
        self.initialpose_interval = float(self.get_parameter('initialpose_interval_sec').value)
        self.wait_subscriber_sec = float(self.get_parameter('initialpose_wait_subscriber_sec').value)
        self.publish_window_sec  = float(self.get_parameter('initialpose_publish_window_sec').value)

        def _args(key: str) -> List[str]:
            v = self.get_parameter(key).value
            return list(v) if isinstance(v, list) else [str(v)]

        # 튜플: (pkg, launch_file, args[])
        self.dr_bringup = (
            self.get_parameter('dr_bringup_pkg').value,
            self.get_parameter('dr_bringup_file').value,
            _args('dr_bringup_args')
        )
        self.dr_local = (
            self.get_parameter('dr_local_pkg').value,
            self.get_parameter('dr_local_file').value,
            _args('dr_local_args')
        )
        self.tr_bringup = (
            self.get_parameter('tr_bringup_pkg').value,
            self.get_parameter('tr_bringup_file').value,
            _args('tr_bringup_args')
        )
        self.tr_local = (
            self.get_parameter('tr_local_pkg').value,
            self.get_parameter('tr_local_file').value,
            _args('tr_local_args')
        )

        # 상태
        self.current_mode: Optional[int] = None
        self.pending: Optional[int] = None
        self.pending_ts_ms: float = 0.0

        # 자식 프로세스 핸들
        self.dr_bringup_p: Optional[subprocess.Popen] = None
        self.dr_local_p:   Optional[subprocess.Popen] = None
        self.tr_bringup_p: Optional[subprocess.Popen] = None
        self.tr_local_p:   Optional[subprocess.Popen] = None

        # AMCL 캐시 & 퍼블리셔
        self.last_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.topic_amcl, self._on_amcl_pose, 10
        )
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.topic_init, 10
        )

        # limit_switch 구독
        self.sub = self.create_subscription(Int32, self.topic_limit, self.on_switch, 10)
        self.get_logger().info(f'watching {self.topic_limit} (Int32)')

        # 초기 모드
        start_mode = int(self.get_parameter('start_mode').value)
        self.switch_to(start_mode)

        # 디바운스 타이머
        self.timer = self.create_timer(0.05, self._tick)

    # ---------- AMCL/initialpose ----------
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_pose = msg

    def _wait_initialpose_subscriber(self, timeout: float) -> bool:
        """AMCL이 initialpose를 구독하기 전이면 유실되므로 구독자 붙을 때까지 대기."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.initialpose_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        return False

    def _publish_initialpose_once(self):
        """캐시된 포즈를 initialpose로 1회 퍼블리시"""
        if self.last_amcl_pose is None:
            self.get_logger().warn('[initialpose] no cached amcl_pose; skip.')
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose = self.last_amcl_pose.pose
        self.initialpose_pub.publish(msg)

    def _publish_initialpose_window(self):
        """초기자세를 지정된 창 동안 반복 퍼블리시 (유실 방지)"""
        if self.last_amcl_pose is None:
            self.get_logger().warn('[initialpose] no cached amcl_pose; skip.')
            return
        self.get_logger().info('[initialpose] publishing cached pose window...')
        t0 = time.time()
        while time.time() - t0 < self.publish_window_sec:
            for _ in range(self.initialpose_repeat):
                self._publish_initialpose_once()
                time.sleep(self.initialpose_interval)
        self.get_logger().info('[initialpose] done.')

    # ---------- 콜백/루프 ----------
    def on_switch(self, msg: Int32):
        desired = 1 if int(msg.data) == 1 else 0
        now_ms = self.get_clock().now().nanoseconds / 1e6
        if self.pending != desired:
            self.pending = desired
            self.pending_ts_ms = now_ms

    def _tick(self):
        if self.pending is None:
            return
        now_ms = self.get_clock().now().nanoseconds / 1e6
        if (now_ms - self.pending_ts_ms) < self.debounce_ms:
            return
        if self.current_mode == self.pending:
            self.pending = None
            return
        self.switch_to(self.pending)
        self.pending = None

    # ---------- 모드 전환 (런치만 갈아끼우기 + 위치 재주입) ----------
    def switch_to(self, mode: int):
        """
        0=Driving, 1=Traction
        - 반대 모드 프로세스 완전 종료(그룹+자식 트리)
        - 대상 모드 bringup 실행 → (유예) → localization 실행
        - localization 후 (유예) → initialpose 구독자 대기 → 퍼블리시 윈도우
        """
        self.get_logger().info(f'=== switching to {"Driving(0)" if mode == 0 else "Traction(1)"} ===')

        # 1) 반대 모드 종료
        if mode == 0:
            terminate_proc(self.tr_local_p,   'traction_localization', self.get_logger())
            terminate_proc(self.tr_bringup_p, 'traction_bringup',      self.get_logger())
        else:
            terminate_proc(self.dr_local_p,   'driving_localization',  self.get_logger())
            terminate_proc(self.dr_bringup_p, 'driving_bringup',       self.get_logger())

        # 2) 리소스 해제 여유 (조금 넉넉히)
        time.sleep(1.0)

        # 3) 대상 모드 실행
        if mode == 0:
            self.dr_bringup_p = popen_launch(*self.dr_bringup)
            time.sleep(self.wait_after_bringup)
            self.dr_local_p = popen_launch(*self.dr_local)
        else:
            self.tr_bringup_p = popen_launch(*self.tr_bringup)
            time.sleep(self.wait_after_bringup)
            self.tr_local_p = popen_launch(*self.tr_local)

        # 4) localization 기동 후 약간 대기
        time.sleep(self.wait_before_initialpose)

        # 5) initialpose 구독자 대기(최대 N초)
        if not self._wait_initialpose_subscriber(self.wait_subscriber_sec):
            self.get_logger().warn('[initialpose] no subscriber (amcl) yet; will publish anyway')

        # 6) 퍼블리시 윈도우 동안 반복 퍼블리시
        self._publish_initialpose_window()

        self.current_mode = mode

    # ---------- 종료 처리 ----------
    def shutdown_children(self):
        self.get_logger().info('shutting down children...')
        terminate_proc(self.dr_local_p,   'driving_localization',  self.get_logger())
        terminate_proc(self.dr_bringup_p, 'driving_bringup',       self.get_logger())
        terminate_proc(self.tr_local_p,   'traction_localization', self.get_logger())
        terminate_proc(self.tr_bringup_p, 'traction_bringup',      self.get_logger())


def main():
    rclpy.init()
    node = ModeProcessManager()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_children()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
