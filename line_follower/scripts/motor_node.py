#!/usr/bin/env python3
import rospy
import pigpio
from geometry_msgs.msg import Twist
import math
import signal, sys

# 핀맵
L_RPWM, L_LPWM, L_REN, L_LEN = 12, 6, 5, 7
R_RPWM, R_LPWM, R_REN, R_LEN = 18, 17, 22, 27

PWM_FREQ_HW = 20000   # 20 kHz (HW PWM: 12,18)
PWM_FREQ_SW = 1000    # 1 kHz  (SW PWM: 6,17)
MAX_DUTY = 1000000    # pigpio hardware PWM duty range (0~1,000,000)
MAX_SW_DUTY = 255     # pigpio software PWM duty range (0~255)

WHEEL_BASE = 0.22     # 바퀴간 거리(m) 대략값
MAX_V = 0.5           # m/s
MAX_W = 3.0           # rad/s

class MotorDriver:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio 연결 실패. pigpiod가 실행 중인지 확인하세요: sudo systemctl start pigpiod")
            sys.exit(1)

        # EN 핀 출력
        for pin in [L_REN, L_LEN, R_REN, R_LEN]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # PWM 핀 모드 & 주파수 설정
        for p in [L_RPWM, R_RPWM]:
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(p, PWM_FREQ_HW)
        for p in [L_LPWM, R_LPWM]:
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(p, PWM_FREQ_SW)

        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd)
        rospy.on_shutdown(self.stop_all)
        self.enable(True)
        self.stop_all()

    def enable(self, en: bool):
        v = 1 if en else 0
        for pin in [L_REN, L_LEN, R_REN, R_LEN]:
            self.pi.write(pin, v)

    def stop_all(self):
        # 모든 PWM 0
        self.pi.hardware_PWM(L_RPWM, PWM_FREQ_HW, 0)
        self.pi.hardware_PWM(R_RPWM, PWM_FREQ_HW, 0)
        self.pi.set_PWM_dutycycle(L_LPWM, 0)
        self.pi.set_PWM_dutycycle(R_LPWM, 0)

    def set_channel(self, rpwm, lpwm, speed):  # speed: -1.0~+1.0
        speed = max(-1.0, min(1.0, speed))
        if speed >= 0:
            # 정방향: RPWM = duty, LPWM = 0
            duty = int(MAX_DUTY * speed)
            self.pi.hardware_PWM(rpwm, PWM_FREQ_HW, duty)
            self.pi.set_PWM_dutycycle(lpwm, 0)
        else:
            # 역방향: LPWM = duty, RPWM = 0
            duty = int(MAX_SW_DUTY * (-speed))
            self.pi.hardware_PWM(rpwm, PWM_FREQ_HW, 0)
            self.pi.set_PWM_dutycycle(lpwm, duty)

    def cb_cmd(self, msg: Twist):
        v = max(-MAX_V, min(MAX_V, msg.linear.x))
        w = max(-MAX_W, min(MAX_W, msg.angular.z))
        # diff-drive 속도 → 좌/우 정규화(-1~1)
        v_l = v - (w * WHEEL_BASE / 2.0)
        v_r = v + (w * WHEEL_BASE / 2.0)
        # 정규화
        m = max(1.0, abs(v_l)/MAX_V, abs(v_r)/MAX_V)
        v_ln = (v_l / (MAX_V*m))
        v_rn = (v_r / (MAX_V*m))
        # 출력
        self.set_channel(L_RPWM, L_LPWM, v_ln)
        self.set_channel(R_RPWM, R_LPWM, v_rn)

if __name__ == "__main__":
    rospy.init_node("motor_node")
    MotorDriver()
    rospy.spin()

