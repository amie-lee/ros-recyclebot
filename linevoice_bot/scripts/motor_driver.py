#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pigpio
import signal
import sys

class BTS7960Driver:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio daemon not running. Start with: sudo systemctl start pigpiod")
            sys.exit(1)

        # Params
        lp = rospy.get_param("~left_motor")
        rp = rospy.get_param("~right_motor")
        pwm = rospy.get_param("~pwm")
        self.duty_max = int(pwm.get("duty_max", 255))
        self.freq_hz  = int(pwm.get("freq_hz", 15000))

        self.left = {
            "RPWM": int(lp["RPWM"]), "LPWM": int(lp["LPWM"]),
            "R_EN": int(lp["R_EN"]), "L_EN": int(lp["L_EN"]),
            "invert": bool(lp.get("invert", False))
        }
        self.right = {
            "RPWM": int(rp["RPWM"]), "LPWM": int(rp["LPWM"]),
            "R_EN": int(rp["R_EN"]), "L_EN": int(rp["L_EN"]),
            "invert": bool(rp.get("invert", False))
        }

        # Setup pins
        for motor in (self.left, self.right):
            for key in ("RPWM","LPWM","R_EN","L_EN"):
                self.pi.set_mode(motor[key], pigpio.OUTPUT)
            # enable both sides
            self.pi.write(motor["R_EN"], 1)
            self.pi.write(motor["L_EN"], 1)
            # PWM freq
            self.pi.set_PWM_frequency(motor["RPWM"], self.freq_hz)
            self.pi.set_PWM_frequency(motor["LPWM"], self.freq_hz)
            self.pi.set_PWM_range(motor["RPWM"], self.duty_max)
            self.pi.set_PWM_range(motor["LPWM"], self.duty_max)
            # stop
            self.pi.set_PWM_dutycycle(motor["RPWM"], 0)
            self.pi.set_PWM_dutycycle(motor["LPWM"], 0)

        self.sub = rospy.Subscriber("cmd_vel", Twist, self.cb_cmd, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def _set_motor_speed(self, motor, speed_norm):
        """
        speed_norm: -1.0 ~ +1.0
        positive -> forward (RPWM), negative -> reverse (LPWM)
        """
        inv = -1.0 if motor["invert"] else 1.0
        s = max(-1.0, min(1.0, speed_norm * inv))
        duty = int(abs(s) * self.duty_max)

        if s > 0:
            # forward
            self.pi.set_PWM_dutycycle(motor["LPWM"], 0)
            self.pi.set_PWM_dutycycle(motor["RPWM"], duty)
        elif s < 0:
            # reverse
            self.pi.set_PWM_dutycycle(motor["RPWM"], 0)
            self.pi.set_PWM_dutycycle(motor["LPWM"], duty)
        else:
            self.pi.set_PWM_dutycycle(motor["RPWM"], 0)
            self.pi.set_PWM_dutycycle(motor["LPWM"], 0)

    def cb_cmd(self, msg):
        # 여기서는 단순 정규화 입력으로 가정 (라인트래커가 0~1 범위로 발행)
        v = msg.linear.x    # -1 ~ +1
        w = msg.angular.z   # -1 ~ +1

        # 간단한 차동 구동
        left = v - w
        right = v + w

        # normalize/clamp
        m = max(1.0, abs(left), abs(right))
        left  /= m
        right /= m

        self._set_motor_speed(self.left, left)
        self._set_motor_speed(self.right, right)

    def shutdown(self):
        # coast
        for motor in (self.left, self.right):
            self.pi.set_PWM_dutycycle(motor["RPWM"], 0)
            self.pi.set_PWM_dutycycle(motor["LPWM"], 0)
            self.pi.write(motor["R_EN"], 0)
            self.pi.write(motor["L_EN"], 0)
        self.pi.stop()

if __name__ == "__main__":
    rospy.init_node("motor_driver")
    BTS7960Driver()
    rospy.spin()

