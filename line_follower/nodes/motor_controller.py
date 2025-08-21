#!/usr/bin/env python3
import rospy, pigpio
from geometry_msgs.msg import Twist

class MotorController:
    def __init__(self):
        # 핀 번호
        self.left_dir1 = rospy.get_param("~left_dir1", 5)
        self.left_dir2 = rospy.get_param("~left_dir2", 6)
        self.left_pwm  = rospy.get_param("~left_pwm", 12)
        self.right_dir1= rospy.get_param("~right_dir1", 13)
        self.right_dir2= rospy.get_param("~right_dir2", 19)
        self.right_pwm = rospy.get_param("~right_pwm", 18)

        self.pi = pigpio.pi()
        for p in [self.left_dir1,self.left_dir2,self.left_pwm,
                  self.right_dir1,self.right_dir2,self.right_pwm]:
            self.pi.set_mode(p, pigpio.OUTPUT)

        self.max_pwm=255
        rospy.Subscriber("cmd_vel", Twist, self.cb)

    def cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        left = max(min(v-w,1.0),-1.0)
        right= max(min(v+w,1.0),-1.0)
        self.set_motor("left", left)
        self.set_motor("right",right)

    def set_motor(self, side, val):
        if side=="left":
            d1,d2,pwm = self.left_dir1,self.left_dir2,self.left_pwm
        else:
            d1,d2,pwm = self.right_dir1,self.right_dir2,self.right_pwm
        duty = int(abs(val)*self.max_pwm)
        if val>=0:
            self.pi.write(d1,1); self.pi.write(d2,0)
        else:
            self.pi.write(d1,0); self.pi.write(d2,1)
        self.pi.set_PWM_dutycycle(pwm,duty)

if __name__=="__main__":
    rospy.init_node("motor_controller")
    MotorController()
    rospy.spin()

