#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VelErrorPlotter (ROS 2 Humble, shark_verification)
- 터틀봇3 수동조종키 스타일로 '목표 선속도 v, 각속도 w'를 키보드로 생성/발행(/<ns>/cmd_vel)
- 현재값은 /<ns>/odom 의 twist.twist.linear.x, angular.z 를 사용
- v, w 및 오차(e_v, e_w)를 실시간 그래프로 시각화

키보드 조작(영문자 키):
  선속도 v:     i(↑), ,(콤마: ↓), k(0으로)
  각속도 w:     j(좌+), l(우-),  m(0으로)
  비상정지:     SPACE (v=w=0)
  스텝/최대값:  [=vstep↓] a(작게), s(크게) / z(작게), x(크게)
  종료:         q

그래프:
  상단:  v_target(파랑 실선) vs v_current(파랑 점선)
  중단:  w_target(초록 실선) vs w_current(초록 점선)
  하단:  |e_v|(빨강), |e_w|(주황)

고정 축 범위:
  시간 축: 최근 window_sec 초만 표시 (기본 30s)
  v축: [-vmax*1.2, +vmax*1.2], w축: [-wmax*1.2, +wmax*1.2]
"""

import os
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 무헤드 환경 대응
import matplotlib
if os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


class VelErrorPlotter(Node):
    def __init__(self):
        super().__init__('vel_error_plotter')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('namespace', 'shark1')     # shark0 | shark1
        self.declare_parameter('v_max', 0.30)             # m/s
        self.declare_parameter('w_max', 1.50)             # rad/s
        self.declare_parameter('v_step', 0.03)            # m/s per key
        self.declare_parameter('w_step', 0.15)            # rad/s per key
        self.declare_parameter('pub_hz', 20.0)            # cmd_vel publish rate
        self.declare_parameter('window_sec', 30.0)        # plot window seconds
        self.declare_parameter('save_fig_path', '')       # Agg일 때 PNG 저장 경로(옵션)
        # -----------------------------------------

        ns           = self.get_parameter('namespace').get_parameter_value().string_value
        self.v_max   = float(self.get_parameter('v_max').get_parameter_value().double_value)
        self.w_max   = float(self.get_parameter('w_max').get_parameter_value().double_value)
        self.v_step  = float(self.get_parameter('v_step').get_parameter_value().double_value)
        self.w_step  = float(self.get_parameter('w_step').get_parameter_value().double_value)
        self.pub_hz  = float(self.get_parameter('pub_hz').get_parameter_value().double_value)
        self.win_sec = float(self.get_parameter('window_sec').get_parameter_value().double_value)
        self.save_png = self.get_parameter('save_fig_path').get_parameter_value().string_value

        # 토픽
        self.topic_cmd  = f'/{ns}/cmd_vel'
        self.topic_odom = f'/{ns}/odom'

        # 퍼블리셔/서브스크라이버
        self.pub_cmd  = self.create_publisher(Twist, self.topic_cmd, 10)
        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self.cb_odom, 50)

        # 타깃/현재/오차 버퍼
        self.t0 = time.time()
        maxlen = int(self.win_sec * 50)  # 충분히 넉넉히 잡고 시간으로 잘라냄
        self.ts = deque(maxlen=maxlen)

        self.v_tgt = deque(maxlen=maxlen)
        self.w_tgt = deque(maxlen=maxlen)
        self.v_cur = deque(maxlen=maxlen)
        self.w_cur = deque(maxlen=maxlen)
        self.e_v   = deque(maxlen=maxlen)
        self.e_w   = deque(maxlen=maxlen)

        # 목표 속도 (키보드로 제어)
        self._lock = threading.Lock()
        self._v_cmd = 0.0
        self._w_cmd = 0.0

        # cmd_vel 주기 퍼블리시 타이머
        self.create_timer(1.0/max(1e-3, self.pub_hz), self.on_pub_cmd)

        # Matplotlib Figure
        self.fig, (self.ax_v, self.ax_w, self.ax_e) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        self.fig.canvas.manager.set_window_title(f"VelErrorPlotter - {ns}")

        # 선 정의
        (self.line_v_tgt,) = self.ax_v.plot([], [], color='#0072BD', lw=1.8, ls='-',  label='v_target [m/s]')
        (self.line_v_cur,) = self.ax_v.plot([], [], color='#0072BD', lw=1.2, ls='--', label='v_current [m/s]')

        (self.line_w_tgt,) = self.ax_w.plot([], [], color='#009E73', lw=1.8, ls='-',  label='w_target [rad/s]')
        (self.line_w_cur,) = self.ax_w.plot([], [], color='#009E73', lw=1.2, ls='--', label='w_current [rad/s]')

        (self.line_e_v,)   = self.ax_e.plot([], [], color='#D55E00', lw=1.6, label='|e_v| [m/s]')
        (self.line_e_w,)   = self.ax_e.plot([], [], color='#EDB120', lw=1.6, label='|e_w| [rad/s]')

        # 축/범례
        pad = 1.2
        self.ax_v.set_ylabel('v [m/s]')
        self.ax_v.set_ylim(-self.v_max*pad, self.v_max*pad)
        self.ax_v.grid(True, linestyle='--', alpha=0.4)
        self.ax_v.legend(loc='upper left')

        self.ax_w.set_ylabel('w [rad/s]')
        self.ax_w.set_ylim(-self.w_max*pad, self.w_max*pad)
        self.ax_w.grid(True, linestyle='--', alpha=0.4)
        self.ax_w.legend(loc='upper left')

        self.ax_e.set_xlabel('time [s]')
        self.ax_e.set_ylabel('abs error')
        self.ax_e.grid(True, linestyle='--', alpha=0.4)
        self.ax_e.legend(loc='upper left')

        # GUI 타이머(메인 스레드에서 주기 갱신)
        self.gui_timer = self.fig.canvas.new_timer(interval=int(1000/10))  # 10 Hz
        self.gui_timer.add_callback(self.update_plot)
        self.gui_timer.start()

        # 키보드 핸들러 등록 (matplotlib 창에 포커스 필요)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.get_logger().info(
            f"[VelErrorPlotter] ns={ns}, pub:{self.topic_cmd}, odom:{self.topic_odom}, vmax={self.v_max}, wmax={self.w_max}"
        )

    # ---------------- ROS 콜백 ----------------
    def cb_odom(self, msg: Odometry):
        # 현재 속도 추정 (odom 기준)
        v = float(msg.twist.twist.linear.x)
        w = float(msg.twist.twist.angular.z)
        with self._lock:
            v_cmd = self._v_cmd
            w_cmd = self._w_cmd

        t = time.time() - self.t0
        self.ts.append(t)

        self.v_cur.append(v)
        self.w_cur.append(w)
        self.v_tgt.append(v_cmd)
        self.w_tgt.append(w_cmd)

        self.e_v.append(abs(v - v_cmd))
        self.e_w.append(abs(w - w_cmd))

        # window_sec 유지(시간으로 trim)
        self._trim_window()

    def _trim_window(self):
        # 시간 창 유지: 맨 앞 시간과 현재 시간 비교
        while self.ts and (self.ts[-1] - self.ts[0] > self.win_sec):
            self.ts.popleft()
            self.v_cur.popleft(); self.w_cur.popleft()
            self.v_tgt.popleft(); self.w_tgt.popleft()
            self.e_v.popleft();   self.e_w.popleft()

    # --------------- cmd_vel 퍼블리시 ---------------
    def on_pub_cmd(self):
        with self._lock:
            v = max(-self.v_max, min(self.v_max, self._v_cmd))
            w = max(-self.w_max, min(self.w_max, self._w_cmd))
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    # --------------- 키보드 제어 ---------------
    def on_key(self, event):
        if event.key is None:
            return
        k = event.key.lower()
        with self._lock:
            if k == 'i':      # v up
                self._v_cmd = min(self._v_cmd + self.v_step, self.v_max)
            elif k == ',':    # v down (comma)
                self._v_cmd = max(self._v_cmd - self.v_step, -self.v_max)
            elif k == 'k':    # v zero
                self._v_cmd = 0.0
                self._w_cmd = 0.0
            elif k == 'j':    # w left (+)
                self._w_cmd = min(self._w_cmd + self.w_step, self.w_max)
            elif k == 'l':    # w right (-)
                self._w_cmd = max(self._w_cmd - self.w_step, -self.w_max)
            elif k == 'm':    # w zero
                self._w_cmd = 0.0

            elif k == ' ':    # space: emergency stop
                self._v_cmd = 0.0
                self._w_cmd = 0.0

            # 스텝 크기 조절
            elif k == 'a':    # v_step smaller
                self.v_step = max(0.005, self.v_step * 0.8)
            elif k == 's':    # v_step bigger
                self.v_step = min(self.v_max, self.v_step * 1.25)
            elif k == 'z':    # w_step smaller
                self.w_step = max(0.02, self.w_step * 0.8)
            elif k == 'x':    # w_step bigger
                self.w_step = min(self.w_max, self.w_step * 1.25)

            elif k == 'q':    # quit
                plt.close(self.fig)
                return

        self.get_logger().info(f"target v={self._v_cmd:.3f} m/s, w={self._w_cmd:.3f} rad/s (steps v={self.v_step:.3f}, w={self.w_step:.3f})")

    # --------------- 플롯 갱신 ---------------
    def update_plot(self):
        if len(self.ts) < 2:
            return
        t0 = self.ts[0]
        tt = [t - t0 for t in self.ts]  # 0부터 시작하는 상대시간으로 표시

        # v
        self.line_v_tgt.set_data(tt, list(self.v_tgt))
        self.line_v_cur.set_data(tt, list(self.v_cur))
        self.ax_v.set_xlim(0, max(1.0, tt[-1]))

        # w
        self.line_w_tgt.set_data(tt, list(self.w_tgt))
        self.line_w_cur.set_data(tt, list(self.w_cur))
        self.ax_w.set_xlim(0, max(1.0, tt[-1]))

        # errors
        self.line_e_v.set_data(tt, list(self.e_v))
        self.line_e_w.set_data(tt, list(self.e_w))
        # 에러 y축은 자동 스케일(상단 10% 마진)
        if self.e_v and self.e_w:
            ymax = max(max(self.e_v), max(self.e_w), 0.05)
            self.ax_e.set_ylim(0, ymax * 1.1)
        self.ax_e.set_xlim(0, max(1.0, tt[-1]))

        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

        # 무헤드 저장 모드
        if matplotlib.get_backend().lower() == 'agg' and self.save_png:
            try:
                self.fig.savefig(self.save_png, dpi=200, bbox_inches='tight')
            except Exception:
                pass


def main():
    rclpy.init()
    node = VelErrorPlotter()

    # ROS 스핀은 백그라운드로, GUI는 메인 스레드
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VelErrorPlotter (ROS 2 Humble, shark_verification)
- 터틀봇3 수동조종키 스타일로 '목표 선속도 v, 각속도 w'를 키보드로 생성/발행(/<ns>/cmd_vel)
- 현재값은 /<ns>/odom 의 twist.twist.linear.x, angular.z 를 사용
- v, w 및 오차(e_v, e_w)를 실시간 그래프로 시각화

키보드 조작(영문자 키):
  선속도 v:     i(↑), ,(콤마: ↓), k(0으로)
  각속도 w:     j(좌+), l(우-),  m(0으로)
  비상정지:     SPACE (v=w=0)
  스텝/최대값:  [=vstep↓] a(작게), s(크게) / z(작게), x(크게)
  종료:         q

그래프:
  상단:  v_target(파랑 실선) vs v_current(파랑 점선)
  중단:  w_target(초록 실선) vs w_current(초록 점선)
  하단:  |e_v|(빨강), |e_w|(주황)

고정 축 범위:
  시간 축: 최근 window_sec 초만 표시 (기본 30s)
  v축: [-vmax*1.2, +vmax*1.2], w축: [-wmax*1.2, +wmax*1.2]
"""

import os
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 무헤드 환경 대응
import matplotlib
if os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


class VelErrorPlotter(Node):
    def __init__(self):
        super().__init__('vel_error_plotter')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('namespace', 'shark1')     # shark0 | shark1
        self.declare_parameter('v_max', 0.30)             # m/s
        self.declare_parameter('w_max', 1.50)             # rad/s
        self.declare_parameter('v_step', 0.03)            # m/s per key
        self.declare_parameter('w_step', 0.15)            # rad/s per key
        self.declare_parameter('pub_hz', 20.0)            # cmd_vel publish rate
        self.declare_parameter('window_sec', 30.0)        # plot window seconds
        self.declare_parameter('save_fig_path', '')       # Agg일 때 PNG 저장 경로(옵션)
        # -----------------------------------------

        ns           = self.get_parameter('namespace').get_parameter_value().string_value
        self.v_max   = float(self.get_parameter('v_max').get_parameter_value().double_value)
        self.w_max   = float(self.get_parameter('w_max').get_parameter_value().double_value)
        self.v_step  = float(self.get_parameter('v_step').get_parameter_value().double_value)
        self.w_step  = float(self.get_parameter('w_step').get_parameter_value().double_value)
        self.pub_hz  = float(self.get_parameter('pub_hz').get_parameter_value().double_value)
        self.win_sec = float(self.get_parameter('window_sec').get_parameter_value().double_value)
        self.save_png = self.get_parameter('save_fig_path').get_parameter_value().string_value

        # 토픽
        self.topic_cmd  = f'/{ns}/cmd_vel'
        self.topic_odom = f'/{ns}/odom'

        # 퍼블리셔/서브스크라이버
        self.pub_cmd  = self.create_publisher(Twist, self.topic_cmd, 10)
        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self.cb_odom, 50)

        # 타깃/현재/오차 버퍼
        self.t0 = time.time()
        maxlen = int(self.win_sec * 50)  # 충분히 넉넉히 잡고 시간으로 잘라냄
        self.ts = deque(maxlen=maxlen)

        self.v_tgt = deque(maxlen=maxlen)
        self.w_tgt = deque(maxlen=maxlen)
        self.v_cur = deque(maxlen=maxlen)
        self.w_cur = deque(maxlen=maxlen)
        self.e_v   = deque(maxlen=maxlen)
        self.e_w   = deque(maxlen=maxlen)

        # 목표 속도 (키보드로 제어)
        self._lock = threading.Lock()
        self._v_cmd = 0.0
        self._w_cmd = 0.0

        # cmd_vel 주기 퍼블리시 타이머
        self.create_timer(1.0/max(1e-3, self.pub_hz), self.on_pub_cmd)

        # Matplotlib Figure
        self.fig, (self.ax_v, self.ax_w, self.ax_e) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        self.fig.canvas.manager.set_window_title(f"VelErrorPlotter - {ns}")

        # 선 정의
        (self.line_v_tgt,) = self.ax_v.plot([], [], color='#0072BD', lw=1.8, ls='-',  label='v_target [m/s]')
        (self.line_v_cur,) = self.ax_v.plot([], [], color='#0072BD', lw=1.2, ls='--', label='v_current [m/s]')

        (self.line_w_tgt,) = self.ax_w.plot([], [], color='#009E73', lw=1.8, ls='-',  label='w_target [rad/s]')
        (self.line_w_cur,) = self.ax_w.plot([], [], color='#009E73', lw=1.2, ls='--', label='w_current [rad/s]')

        (self.line_e_v,)   = self.ax_e.plot([], [], color='#D55E00', lw=1.6, label='|e_v| [m/s]')
        (self.line_e_w,)   = self.ax_e.plot([], [], color='#EDB120', lw=1.6, label='|e_w| [rad/s]')

        # 축/범례
        pad = 1.2
        self.ax_v.set_ylabel('v [m/s]')
        self.ax_v.set_ylim(-self.v_max*pad, self.v_max*pad)
        self.ax_v.grid(True, linestyle='--', alpha=0.4)
        self.ax_v.legend(loc='upper left')

        self.ax_w.set_ylabel('w [rad/s]')
        self.ax_w.set_ylim(-self.w_max*pad, self.w_max*pad)
        self.ax_w.grid(True, linestyle='--', alpha=0.4)
        self.ax_w.legend(loc='upper left')

        self.ax_e.set_xlabel('time [s]')
        self.ax_e.set_ylabel('abs error')
        self.ax_e.grid(True, linestyle='--', alpha=0.4)
        self.ax_e.legend(loc='upper left')

        # GUI 타이머(메인 스레드에서 주기 갱신)
        self.gui_timer = self.fig.canvas.new_timer(interval=int(1000/10))  # 10 Hz
        self.gui_timer.add_callback(self.update_plot)
        self.gui_timer.start()

        # 키보드 핸들러 등록 (matplotlib 창에 포커스 필요)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.get_logger().info(
            f"[VelErrorPlotter] ns={ns}, pub:{self.topic_cmd}, odom:{self.topic_odom}, vmax={self.v_max}, wmax={self.w_max}"
        )

    # ---------------- ROS 콜백 ----------------
    def cb_odom(self, msg: Odometry):
        # 현재 속도 추정 (odom 기준)
        v = float(msg.twist.twist.linear.x)
        w = float(msg.twist.twist.angular.z)
        with self._lock:
            v_cmd = self._v_cmd
            w_cmd = self._w_cmd

        t = time.time() - self.t0
        self.ts.append(t)

        self.v_cur.append(v)
        self.w_cur.append(w)
        self.v_tgt.append(v_cmd)
        self.w_tgt.append(w_cmd)

        self.e_v.append(abs(v - v_cmd))
        self.e_w.append(abs(w - w_cmd))

        # window_sec 유지(시간으로 trim)
        self._trim_window()

    def _trim_window(self):
        # 시간 창 유지: 맨 앞 시간과 현재 시간 비교
        while self.ts and (self.ts[-1] - self.ts[0] > self.win_sec):
            self.ts.popleft()
            self.v_cur.popleft(); self.w_cur.popleft()
            self.v_tgt.popleft(); self.w_tgt.popleft()
            self.e_v.popleft();   self.e_w.popleft()

    # --------------- cmd_vel 퍼블리시 ---------------
    def on_pub_cmd(self):
        with self._lock:
            v = max(-self.v_max, min(self.v_max, self._v_cmd))
            w = max(-self.w_max, min(self.w_max, self._w_cmd))
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    # --------------- 키보드 제어 ---------------
    def on_key(self, event):
        if event.key is None:
            return
        k = event.key.lower()
        with self._lock:
            if k == 'i':      # v up
                self._v_cmd = min(self._v_cmd + self.v_step, self.v_max)
            elif k == ',':    # v down (comma)
                self._v_cmd = max(self._v_cmd - self.v_step, -self.v_max)
            elif k == 'k':    # v zero
                self._v_cmd = 0.0

            elif k == 'j':    # w left (+)
                self._w_cmd = min(self._w_cmd + self.w_step, self.w_max)
            elif k == 'l':    # w right (-)
                self._w_cmd = max(self._w_cmd - self.w_step, -self.w_max)
            elif k == 'm':    # w zero
                self._w_cmd = 0.0

            elif k == ' ':    # space: emergency stop
                self._v_cmd = 0.0
                self._w_cmd = 0.0

            # 스텝 크기 조절
            elif k == 'a':    # v_step smaller
                self.v_step = max(0.005, self.v_step * 0.8)
            elif k == 's':    # v_step bigger
                self.v_step = min(self.v_max, self.v_step * 1.25)
            elif k == 'z':    # w_step smaller
                self.w_step = max(0.02, self.w_step * 0.8)
            elif k == 'x':    # w_step bigger
                self.w_step = min(self.w_max, self.w_step * 1.25)

            elif k == 'q':    # quit
                plt.close(self.fig)
                return

        self.get_logger().info(f"target v={self._v_cmd:.3f} m/s, w={self._w_cmd:.3f} rad/s (steps v={self.v_step:.3f}, w={self.w_step:.3f})")

    # --------------- 플롯 갱신 ---------------
    def update_plot(self):
        if len(self.ts) < 2:
            return
        t0 = self.ts[0]
        tt = [t - t0 for t in self.ts]  # 0부터 시작하는 상대시간으로 표시

        # v
        self.line_v_tgt.set_data(tt, list(self.v_tgt))
        self.line_v_cur.set_data(tt, list(self.v_cur))
        self.ax_v.set_xlim(0, max(1.0, tt[-1]))

        # w
        self.line_w_tgt.set_data(tt, list(self.w_tgt))
        self.line_w_cur.set_data(tt, list(self.w_cur))
        self.ax_w.set_xlim(0, max(1.0, tt[-1]))

        # errors
        self.line_e_v.set_data(tt, list(self.e_v))
        self.line_e_w.set_data(tt, list(self.e_w))
        # 에러 y축은 자동 스케일(상단 10% 마진)
        if self.e_v and self.e_w:
            ymax = max(max(self.e_v), max(self.e_w), 0.05)
            self.ax_e.set_ylim(0, ymax * 1.1)
        self.ax_e.set_xlim(0, max(1.0, tt[-1]))

        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

        # 무헤드 저장 모드
        if matplotlib.get_backend().lower() == 'agg' and self.save_png:
            try:
                self.fig.savefig(self.save_png, dpi=200, bbox_inches='tight')
            except Exception:
                pass


def main():
    rclpy.init()
    node = VelErrorPlotter()

    # ROS 스핀은 백그라운드로, GUI는 메인 스레드
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
