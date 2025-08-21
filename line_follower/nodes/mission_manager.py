#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class MissionManager:
    def __init__(self):
        rospy.init_node("mission_manager")
        # 파라미터
        self.rotate_180_time = rospy.get_param("~rotate_180_time", 1.2)   # ★ 현장 보정 필요
        self.rotate_speed    = rospy.get_param("~rotate_speed", 0.6)      # 회전 속도 (|angular.z|)
        self.track_enable_on_start = rospy.get_param("~track_enable_on_start", False)

        # 토픽
        self.pub_enable = rospy.Publisher("/line_follower/enable", Bool, queue_size=1, latch=True)
        self.pub_cmd    = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/voice_cmd", String, self.cb_voice, queue_size=5)
        rospy.Subscriber("/line_follower/lost", Bool, self.cb_lost, queue_size=5)

        self.state = "IDLE"  # IDLE, TRACKING, RETURN_TRACK
        self.last_lost = True
        self.pub_enable.publish(Bool(self.track_enable_on_start))
        rospy.loginfo("mission_manager ready (state=%s)", self.state)

    def cb_voice(self, msg: String):
        if msg.data == "go":
            rospy.loginfo("Voice: GO → start tracking")
            self.start_tracking()
        elif msg.data == "return":
            rospy.loginfo("Voice: RETURN → rotate 180, start tracking, stop at end, rotate 180")
            self.rotate_180()
            self.start_tracking()
            self.state = "RETURN_TRACK"

    def cb_lost(self, msg: Bool):
        self.last_lost = msg.data
        if msg.data and self.state in ("TRACKING", "RETURN_TRACK"):
            rospy.loginfo("Line lost → stop tracking")
            self.pub_enable.publish(Bool(False))
            self.stop_motion()
            if self.state == "RETURN_TRACK":
                rospy.loginfo("RETURN mission: final rotate 180")
                self.rotate_180()
            self.state = "IDLE"

    # helpers
    def start_tracking(self):
        self.state = "TRACKING"
        self.pub_enable.publish(Bool(True))

    def stop_motion(self):
        self.pub_cmd.publish(Twist())  # stop
def rotate_180(self):
    rospy.loginfo("돌아가 명령 → 좌회전 180도 회전 시작")

    # 좌회전: angular.z > 0
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = +1.0   # motor_controller가 회전으로 인식 → duty_turn=50% 적용

    end = rospy.Time.now() + rospy.Duration.from_sec(3.8)  # 3.8초 동안 회전
    r = rospy.Rate(30)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        self.pub_cmd.publish(t)
        r.sleep()

    # 회전 종료 → 정지
    self.stop_motion()
    rospy.loginfo("180도 회전 종료")


if __name__ == "__main__":
    try:
        MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

