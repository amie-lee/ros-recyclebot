#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BTS7960 dual H-bridge 안전 컨트롤러 (ROS1 /cmd_vel 구독)
- 안전: 데드타임, 코스트(양 PWM=0), 램프(가감속 제한), 듀티 상/하한, 워치독
- 듀티 정책: 직진( |v| >= |w| ) → 최대 25%, 회전( |w| > |v| ) → 최대 50%
- 입력: geometry_msgs/Twist (/cmd_vel)  linear.x, angular.z ∈ [-1, 1]
- 핀(BCM) 기본:
    Right (모터1): RPWM=20, LPWM=21, R_EN=23, L_EN=24
    Left  (모터2): RPWM=5,  LPWM=6,  R_EN=17, L_EN=27
"""

import time
import math
import threading
import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Side:
    """한쪽 바퀴(R/L)의 BTS7960 듀얼 PWM 제어"""
    def __init__(self, name, rpwm, lpwm, ren, len_, pwm_hz,
                 min_duty, max_duty, deadtime_s, coast_gap_s, invert=False):
        self.name = name
        self.rpwm_pin = rpwm
        self.lpwm_pin = lpwm
        self.ren_pin = ren
        self.len_pin = len_
        self.min_duty = float(min_duty)
        self.max_duty = float(max_duty)
        self.deadtime_s = float(deadtime_s)
        self.coast_gap_s = float(coast_gap_s)
        self.invert = invert

        GPIO.setup(self.rpwm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.lpwm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ren_pin,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.len_pin,  GPIO.OUT, initial=GPIO.LOW)

        self.p_r = GPIO.PWM(self.rpwm_pin, pwm_hz); self.p_r.start(0)
        self.p_l = GPIO.PWM(self.lpwm_pin, pwm_hz); self.p_l.start(0)

        # Enable
        GPIO.output(self.ren_pin, GPIO.HIGH)
        GPIO.output(self.len_pin, GPIO.HIGH)

        self.cur_dir = 0     # -1, 0, +1
        self.cur_duty = 0.0  # 0~100
        self._lock = threading.Lock()

    def _apply_coast(self):
        self.p_r.ChangeDutyCycle(0)
        self.p_l.ChangeDutyCycle(0)

    def set_command(self, value_norm):
        """
        value_norm ∈ [-1, 1]  (정규화된 목표치)
        - invert 적용
        - 방향 전환 시: 코스트 → 데드타임 → 새 듀티
        - 듀티 하한/상한 적용
        """
        if self.invert:
            value_norm = -value_norm

        direction = 0
        if value_norm > 1e-6: direction = +1
        elif value_norm < -1e-6: direction = -1
        duty = abs(value_norm) * self.max_duty
        if duty > 0 and duty < self.min_duty:
            duty = self.min_duty

        with self._lock:
            # 방향 바뀌면 코스트 + 데드타임
            if direction != 0 and direction != self.cur_dir and (self.cur_duty > 0 or self.cur_dir != 0):
                self._apply_coast()
                time.sleep(self.coast_gap_s)
                time.sleep(self.deadtime_s)

            if direction >= 0:
                # 전진: RPWM만 듀티, LPWM=0
                self.p_l.ChangeDutyCycle(0)
                self.p_r.ChangeDutyCycle(duty if direction != 0 else 0)
            else:
                # 후진: LPWM만 듀티, RPWM=0
                self.p_r.ChangeDutyCycle(0)
                self.p_l.ChangeDutyCycle(duty)

            self.cur_dir = direction
            self.cur_duty = duty if direction != 0 else 0.0

    def stop(self):
        with self._lock:
            self._apply_coast()
            self.p_r.stop()
            self.p_l.stop()
            GPIO.output(self.ren_pin, GPIO.LOW)
            GPIO.output(self.len_pin, GPIO.LOW)


class MotorController:
    def __init__(self):
        rospy.init_node("motor_controller_bts7960")

        # ===== 파라미터 =====
        # 핀(BCM)
        self.RPWM1 = rospy.get_param("~right_rpwm", 24)
        self.LPWM1 = rospy.get_param("~right_lpwm", 17)
        self.R_EN1 = rospy.get_param("~right_ren", 22)
        self.L_EN1 = rospy.get_param("~right_len", 27)

        self.RPWM2 = rospy.get_param("~left_rpwm", 23)
        self.LPWM2 = rospy.get_param("~left_lpwm", 6)
        self.R_EN2 = rospy.get_param("~left_ren", 5)
        self.L_EN2 = rospy.get_param("~left_len", 7)

        # PWM/안전
        self.PWM_HZ = rospy.get_param("~pwm_hz", 1000)              # 1 kHz (조용하게 하려면 pigpio 20 kHz 권장)
        self.DEADTIME_S = rospy.get_param("~deadtime_s", 0.005)     # 5 ms
        self.COAST_GAP_S = rospy.get_param("~coast_gap_s", 0.6)     # 방향 전환 전 코스트 유지
        self.MIN_DUTY = rospy.get_param("~min_duty", 20.0)          # 정지극복
        self.MAX_DUTY = rospy.get_param("~max_duty", 60.0)          # 상한(내부에서 25/50% 선택함)
        self.RAMP_DUTY_PER_S = rospy.get_param("~ramp_duty_per_s", 80.0)  # duty%/s
        self.ANGULAR_GAIN = rospy.get_param("~angular_gain", 1.0)
        self.SAFE_TIMEOUT = rospy.get_param("~safe_timeout", 0.5)   # s (명령 미수신 시 정지 램프)

        self.INVERT_LEFT  = rospy.get_param("~invert_left",  False)
        self.INVERT_RIGHT = rospy.get_param("~invert_right", False)

        # 듀티 정책(요청 사양)
        self.DUTY_STRAIGHT = rospy.get_param("~duty_straight", 25.0)  # 직진
        self.DUTY_TURN     = rospy.get_param("~duty_turn", 50.0)      # 회전

        # GPIO
        GPIO.setmode(GPIO.BCM)

        # 바퀴 객체
        self.right = Side("right", self.RPWM1, self.LPWM1, self.R_EN1, self.L_EN1,
                          self.PWM_HZ, self.MIN_DUTY, self.MAX_DUTY,
                          self.DEADTIME_S, self.COAST_GAP_S, invert=self.INVERT_RIGHT)
        self.left  = Side("left",  self.RPWM2, self.LPWM2, self.R_EN2, self.L_EN2,
                          self.PWM_HZ, self.MIN_DUTY, self.MAX_DUTY,
                          self.DEADTIME_S, self.COAST_GAP_S, invert=self.INVERT_LEFT)

        # 목표/현재 (정규화 -1..1)
        self._tgt_l = 0.0
        self._tgt_r = 0.0
        self._cur_l = 0.0
        self._cur_r = 0.0
        self._last_cmd_time = rospy.get_time()
        self._lock = threading.Lock()

        rospy.Subscriber("cmd_vel", Twist, self._cmd_cb, queue_size=1)

        # 제어 루프(50Hz)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("motor_controller_bts7960 (safe, slow) started")

    # ===== /cmd_vel 콜백 =====
    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = rospy.get_time()
        v = clamp(msg.linear.x, -1.0, 1.0)
        w = clamp(msg.angular.z, -1.0, 1.0)
        k = float(self.ANGULAR_GAIN)

        # 비정규 좌/우(비율 유지)
        raw_l = v - k * w
        raw_r = v + k * w

        # 직진/회전 판별 → 듀티 스케일 결정 (요청값 25% / 50%)
        if abs(v) >= abs(w):
            duty_scale = self.DUTY_STRAIGHT / 100.0   # 0.25
        else:
            duty_scale = self.DUTY_TURN / 100.0       # 0.50

        # 좌/우 비율 유지하며 최대값 1에 맞춰 정규화 → 듀티 스케일 곱
        norm = max(abs(raw_l), abs(raw_r), 1e-6)
        tgt_l = (raw_l / norm) * duty_scale
        tgt_r = (raw_r / norm) * duty_scale

        with self._lock:
            self._tgt_l = clamp(tgt_l, -1.0, 1.0)
            self._tgt_r = clamp(tgt_r, -1.0, 1.0)

    # ===== 램프(가감속 제한) =====
    def _ramp_toward(self, cur, tgt, dt):
        max_step = (self.RAMP_DUTY_PER_S / 100.0) * dt  # duty%/s -> 정규화 스텝
        # 방향 바뀌면 먼저 0으로 수렴
        if math.copysign(1.0, cur) != math.copysign(1.0, tgt) and abs(cur) > 1e-6:
            step = -math.copysign(min(abs(cur), max_step), cur)
            return cur + step
        # 같은 부호면 목표로 수렴
        delta = tgt - cur
        step = clamp(delta, -max_step, max_step)
        return cur + step

    # ===== 제어 루프 =====
    def _loop(self):
        rate_hz = 50.0
        dt = 1.0 / rate_hz
        while self._running and not rospy.is_shutdown():
            now = rospy.get_time()
            # 워치독: 타임아웃이면 0으로 램프다운
            if (now - self._last_cmd_time) > self.SAFE_TIMEOUT:
                with self._lock:
                    self._tgt_l = 0.0
                    self._tgt_r = 0.0

            with self._lock:
                tgt_l, tgt_r = self._tgt_l, self._tgt_r

            self._cur_l = self._ramp_toward(self._cur_l, tgt_l, dt)
            self._cur_r = self._ramp_toward(self._cur_r, tgt_r, dt)

            self.left.set_command(self._cur_l)
            self.right.set_command(self._cur_r)

            time.sleep(dt)

    def shutdown(self):
        self._running = False
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.left.stop()
            self.right.stop()
        finally:
            GPIO.cleanup()
        rospy.loginfo("motor_controller_bts7960 stopped")


if __name__ == "__main__":
    MotorController()
    rospy.spin()

