#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, math
from geometry_msgs.msg import Twist

try:
    import RPi.GPIO as GPIO
except ImportError:
    raise RuntimeError("RPi.GPIO가 설치되어 있지 않습니다. `sudo apt install python3-rpi.gpio`를 실행하세요.")

class BTS7960DiffDriver:
    def __init__(self):
        rospy.init_node("bts7960_diff_driver")

        # ===== 파라미터 =====
        p = rospy.get_param("~pins", {})
        self.RPWM1 = int(p.get("rpwm_right", 24))
        self.LPWM1 = int(p.get("lpwm_right", 17))
        self.R_EN1 = int(p.get("r_en_right", 22))
        self.L_EN1 = int(p.get("l_en_right", 27))

        self.RPWM2 = int(p.get("rpwm_left", 23))
        self.LPWM2 = int(p.get("lpwm_left", 6))
        self.R_EN2 = int(p.get("r_en_left", 5))
        self.L_EN2 = int(p.get("l_en_left", 18))

        self.PWM_HZ     = float(rospy.get_param("~pwm_hz", 1000.0))   # 1kHz
        self.DEADTIME_S = float(rospy.get_param("~deadtime_s", 0.005))
        self.MAX_DUTY   = float(rospy.get_param("~max_duty", 60.0))   # [%]
        self.MIN_DUTY   = float(rospy.get_param("~min_duty", 15.0))   # [%] 최소 구동 듀티 (정지 마찰 극복)
        self.DEADZONE   = float(rospy.get_param("~deadzone", 0.02))   # |u|<deadzone이면 0으로 간주

        # Twist → 듀티 스케일
        self.MAX_LINEAR  = float(rospy.get_param("~max_linear", 0.5))   # m/s 가정치
        self.MAX_ANGULAR = float(rospy.get_param("~max_angular", 1.5))  # rad/s 가정치

        # 배선 극성 보정
        self.INVERT_LEFT  = bool(rospy.get_param("~invert_left", False))
        self.INVERT_RIGHT = bool(rospy.get_param("~invert_right", False))

        self.CMD_TIMEOUT = float(rospy.get_param("~cmd_timeout", 0.5))  # 초

        # ===== GPIO 초기화 =====
        GPIO.setmode(GPIO.BCM)
        for pin in (self.RPWM1, self.LPWM1, self.R_EN1, self.L_EN1,
                    self.RPWM2, self.LPWM2, self.R_EN2, self.L_EN2):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # EN enable
        GPIO.output(self.R_EN1, GPIO.HIGH); GPIO.output(self.L_EN1, GPIO.HIGH)
        GPIO.output(self.R_EN2, GPIO.HIGH); GPIO.output(self.L_EN2, GPIO.HIGH)

        self.p_r1 = GPIO.PWM(self.RPWM1, int(self.PWM_HZ)); self.p_r1.start(0)
        self.p_l1 = GPIO.PWM(self.LPWM1, int(self.PWM_HZ)); self.p_l1.start(0)
        self.p_r2 = GPIO.PWM(self.RPWM2, int(self.PWM_HZ)); self.p_r2.start(0)
        self.p_l2 = GPIO.PWM(self.LPWM2, int(self.PWM_HZ)); self.p_l2.start(0)

        # 내부 상태
        self.last_cmd_time = 0.0
        self.target_lin = 0.0
        self.target_ang = 0.0

        # 이전 출력 부호(데드타임 판단용): +1, 0, -1
        self.sign_left  = 0
        self.sign_right = 0

        # 통신
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.on_timer)  # 50Hz

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("bts7960_diff_driver started")

    # ===== 유틸 =====
    @staticmethod
    def _sign(x):
        if x > 0: return 1
        if x < 0: return -1
        return 0

    def _coast_side(self, p_r, p_l):
        p_r.ChangeDutyCycle(0); p_l.ChangeDutyCycle(0)

    def _backward_side(self, p_r, p_l, duty):
        # 역회전: R핀 사용
        p_l.ChangeDutyCycle(0)
        time.sleep(self.DEADTIME_S)
        p_r.ChangeDutyCycle(duty)

    def _forward_side(self, p_r, p_l, duty):
        # 정회전: L핀 사용
        p_r.ChangeDutyCycle(0)
        time.sleep(self.DEADTIME_S)
        p_l.ChangeDutyCycle(duty)

    def _apply_side(self, duty, p_r, p_l, prev_sign_store):
        """한쪽 모터에 duty(부호 포함)를 안전 적용. prev_sign_store는 참조/갱신용 리스트 [sign]"""
        prev_sign = prev_sign_store[0]
        s = self._sign(duty)
        a = abs(duty)

        if s == 0 or a == 0:
            # 코스트
            self._coast_side(p_r, p_l)
            prev_sign_store[0] = 0
            return

        # 데드존 → 0
        if a < self.MIN_DUTY:
            a = self.MIN_DUTY

        # 부호가 바뀌면 데드타임 처리
        if prev_sign != 0 and s != prev_sign:
            self._coast_side(p_r, p_l)
            time.sleep(self.DEADTIME_S)

        if s > 0:
            self._forward_side(p_r, p_l, a)
        else:
            self._backward_side(p_r, p_l, a)

        prev_sign_store[0] = s

    def _mix(self, lin, ang):
        """Twist → 좌/우 무차원 u(-1~1) → 듀티[%]"""
        # 범위를 넘어오면 saturate
        u_lin = max(-1.0, min(1.0, lin / self.MAX_LINEAR)) if self.MAX_LINEAR > 0 else 0.0
        u_ang = max(-1.0, min(1.0, ang / self.MAX_ANGULAR)) if self.MAX_ANGULAR > 0 else 0.0

        u_left  = u_lin + u_ang
        u_right = u_lin - u_ang

        # 정규화 (최대 1.0)
        m = max(1.0, abs(u_left), abs(u_right))
        u_left  /= m
        u_right /= m

        # 데드존
        if abs(u_left)  < self.DEADZONE:  u_left  = 0.0
        if abs(u_right) < self.DEADZONE:  u_right = 0.0

        # 극성 보정
        if self.INVERT_LEFT:  u_left  = -u_left
        if self.INVERT_RIGHT: u_right = -u_right

        # 듀티 변환
        d_left  = u_left  * self.MAX_DUTY
        d_right = u_right * self.MAX_DUTY
        return d_left, d_right

    # ===== 콜백/루프 =====
    def cb_cmd(self, msg: Twist):
        self.target_lin = float(msg.linear.x)
        self.target_ang = float(msg.angular.z)
        self.last_cmd_time = rospy.get_time()

    def on_timer(self, _evt):
        now = rospy.get_time()
        # 타임아웃 → 정지(coast)
        if now - self.last_cmd_time > self.CMD_TIMEOUT:
            self._apply_side(0, self.p_r1, self.p_l1, [self.sign_right])
            self._apply_side(0, self.p_r2, self.p_l2, [self.sign_left])
            self.sign_right = 0; self.sign_left = 0
            return

        d_left, d_right = self._mix(self.target_lin, self.target_ang)

        # 좌/우 적용 (prev_sign 저장은 리스트로 전달하여 내부에서 갱신받기)
        left_store  = [self.sign_left]
        right_store = [self.sign_right]

        self._apply_side(d_right, self.p_r1, self.p_l1, right_store)  # 오른쪽 모터(모터1)
        self._apply_side(d_left,  self.p_r2, self.p_l2, left_store)   # 왼쪽 모터(모터2)

        self.sign_right = right_store[0]
        self.sign_left  = left_store[0]

    def on_shutdown(self):
        try:
            self._coast_side(self.p_r1, self.p_l1)
            self._coast_side(self.p_r2, self.p_l2)
            self.p_r1.stop(); self.p_l1.stop(); self.p_r2.stop(); self.p_l2.stop()
        except Exception:
            pass
        GPIO.cleanup()
        rospy.loginfo("bts7960_diff_driver stopped")

if __name__ == "__main__":
    BTS7960DiffDriver()
    rospy.spin()
