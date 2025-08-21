#!/usr/bin/env python3
import rospy, pigpio
from std_msgs.msg import Float32

SERVO_PIN = 13  # PWM1
# 마이크로초 펄스폭 기준(전형적 500~2500us)
MIN_US, MAX_US = 500, 2500

class ServoNode:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.signal_shutdown("pigpio not connected")
            return
        rospy.Subscriber("/servo/angle", Float32, self.cb_angle)
        rospy.on_shutdown(self.shutdown)
        # 기본 중립
        self.set_angle(90.0)

    def angle_to_us(self, deg):
        deg = max(0.0, min(180.0, deg))
        return int(MIN_US + (MAX_US - MIN_US) * (deg/180.0))

    def set_angle(self, deg):
        us = self.angle_to_us(deg)
        self.pi.set_servo_pulsewidth(SERVO_PIN, us)

    def cb_angle(self, msg):
        self.set_angle(msg.data)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0)
        self.pi.stop()

if __name__ == "__main__":
    rospy.init_node("servo_node")
    ServoNode()
    rospy.spin()

