#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pigpio
import time
import math

# ===== ¼³Á¤ =====
SERVO_GPIO = 18      # PWM °¡´ÉÇÑ ÇÉ(¿¹: 12, 13, 18, 19)
NEUTRAL_DEG = 90
PLASTIC_DEG = 30
CAN_DEG = 150

# ¼­º¸ º¸Á¤
MIN_USEC = 500
MAX_USEC = 2500

def deg_to_usec(deg):
    deg = max(0, min(180, deg))
    return int(MIN_USEC + (MAX_USEC - MIN_USEC) * (deg / 180.0))

class ServoController:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio daemon not running. Try: sudo systemctl enable --now pigpiod")
            raise RuntimeError("pigpio not connected")
        self.move_to(NEUTRAL_DEG)
        rospy.Subscriber("/bin_command", String, self.cb)

    def move_to(self, deg):
        pw = deg_to_usec(deg)
        self.pi.set_servo_pulsewidth(SERVO_GPIO, pw)
        rospy.loginfo("Servo -> %d deg (%dus)", deg, pw)

    def cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "plastic":
            self.move_to(PLASTIC_DEG)
        elif cmd == "can":
            self.move_to(CAN_DEG)
        else:
            rospy.logwarn("Unknown command: %s", cmd)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(SERVO_GPIO, 0)
        self.pi.stop()

def main():
    rospy.init_node("servo_controller")
    sc = ServoController()
    rospy.on_shutdown(sc.shutdown)
    rospy.loginfo("servo_controller ready on GPIO %d", SERVO_GPIO)
    rospy.spin()

if __name__ == "__main__":
    main()

