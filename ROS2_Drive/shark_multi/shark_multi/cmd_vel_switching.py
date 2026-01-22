#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

EPS = 1e-6

class Mode:
    IDLE = 0
    TRANSLATE = 1
    ROTATE = 2

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

class DecoupledCmdVel(Node):
    """
    입력:  in_topic  (기본: /shark1/cmd_vel)
    출력:  out_topic (기본: /shark1/cmd_vel_switch)

    동작:
      - v,w 동시 명령이 들어오면 규칙에 따라 '직진 모드' 또는 '회전 모드'로 택일
      - 모드 전환 시 짧게 정지(stop_before_switch_s)하여 슬립/채터링 완화
      - hold_time_s 동안 모드 유지(빠른 스위칭 방지)
    """
    def __init__(self):
        super().__init__('cmd_vel_switching')

        # ---- Parameters (요청 기본값 반영) ----
        self.declare_parameter('in_topic', '/shark1/cmd_vel')
        self.declare_parameter('out_topic', '/shark1/cmd_vel_switch')

        self.declare_parameter('corner_ratio', 1.4)            # |w| / (|v|+ε) > corner_ratio → 회전 우선
        self.declare_parameter('prioritize_translation', True) # 애매하면 직진 우선 여부
        self.declare_parameter('hold_time_s', 0.35)            # 모드 전환 후 유지 시간
        self.declare_parameter('stop_before_switch_s', 0.10)   # 전환 직전 잠깐 정지
        self.declare_parameter('lin_deadband', 0.01)           # 미세 직진 제거
        self.declare_parameter('ang_deadband', 0.01)           # 미세 회전 제거
        self.declare_parameter('max_lin_when_rotating', 0.0)   # 회전 중 허용 직진(권장 0)
        self.declare_parameter('max_ang_when_translating', 0.0)# 직진 중 허용 회전(권장 0)

        # ---- Load parameters ----
        self.in_topic  = self.get_parameter('in_topic').get_parameter_value().string_value
        self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value

        self.corner_ratio = self.get_parameter('corner_ratio').get_parameter_value().double_value
        self.prioritize_translation = self.get_parameter('prioritize_translation').get_parameter_value().bool_value
        self.hold_time_s  = self.get_parameter('hold_time_s').get_parameter_value().double_value
        self.stop_before_switch_s = self.get_parameter('stop_before_switch_s').get_parameter_value().double_value
        self.lin_deadband = self.get_parameter('lin_deadband').get_parameter_value().double_value
        self.ang_deadband = self.get_parameter('ang_deadband').get_parameter_value().double_value
        self.max_lin_when_rotating = self.get_parameter('max_lin_when_rotating').get_parameter_value().double_value
        self.max_ang_when_translating = self.get_parameter('max_ang_when_translating').get_parameter_value().double_value

        # ---- I/O ----
        self.sub = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, self.out_topic, 10)

        # ---- State ----
        self.mode = Mode.IDLE
        self.last_switch = 0.0

        self.get_logger().info(
            f"[DecoupledCmdVel] in='{self.in_topic}' → out='{self.out_topic}' "
            f"(corner_ratio={self.corner_ratio}, hold={self.hold_time_s}s, stop_before={self.stop_before_switch_s}s)"
        )

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def stop_output(self):
        self.pub.publish(Twist())

    def switch_mode(self, new_mode: int):
        if new_mode != self.mode:
            if self.stop_before_switch_s > 0.0:
                self.stop_output()
                # 주의: 콜백 내 sleep은 그 시점의 명령 스트림만 잠깐 멈춥니다.
                # 실시간성이 더 필요하면 타이머 기반으로 바꾸세요.
                time.sleep(self.stop_before_switch_s)
            self.mode = new_mode
            self.last_switch = self.now()

    def hold_active(self) -> bool:
        return (self.now() - self.last_switch) < self.hold_time_s

    def decide_mode(self, v: float, w: float) -> int:
        av = abs(v); aw = abs(w)
        if av < self.lin_deadband and aw < self.ang_deadband:
            return Mode.IDLE
        ratio = aw / (av + EPS)
        if ratio > self.corner_ratio:
            return Mode.ROTATE
        elif (1.0 / (ratio + EPS)) > self.corner_ratio:
            return Mode.TRANSLATE
        else:
            return Mode.TRANSLATE if self.prioritize_translation else Mode.ROTATE

    def on_cmd(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # 모드 결정(hold 시간 동안은 기존 모드 유지)
        new_mode = self.decide_mode(v, w) if not self.hold_active() else self.mode
        self.switch_mode(new_mode)

        out = Twist()

        if self.mode == Mode.IDLE:
            if abs(v) < self.lin_deadband: v = 0.0
            if abs(w) < self.ang_deadband: w = 0.0
            out.linear.x = v
            out.angular.z = w

        elif self.mode == Mode.TRANSLATE:
            out.linear.x = v
            out.angular.z = clamp(w, -self.max_ang_when_translating, self.max_ang_when_translating)

        elif self.mode == Mode.ROTATE:
            out.angular.z = w
            out.linear.x = clamp(v, -self.max_lin_when_rotating, self.max_lin_when_rotating)

        self.pub.publish(out)

def main():
    rclpy.init()
    node = DecoupledCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
