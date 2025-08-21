#!/usr/bin/env python3
import rospy, math, time
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class StateNode:
    def __init__(self):
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/voice/command", String, self.cb_voice)
        rospy.Subscriber("/line_tracker/error", Float32, self.cb_err)
        rospy.Subscriber("/line_tracker/status", String, self.cb_status)

        self.state = "IDLE"   # IDLE, TRACKING, ROTATING, RETURN_SEQUENCE
        self.last_err = 0.0
        self.Kp = rospy.get_param("~kp", 1.2)
        self.v_track = rospy.get_param("~v_track", 0.2)
        self.w_rotate = rospy.get_param("~w_rotate", 2.0)  # rad/s
        self.rotate_time_180 = math.pi / abs(self.w_rotate) + 0.3  # 여유

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

        self.return_phase = 0  # 0: 회전, 1: 트래킹, 2: 회전끝

    def cb_voice(self, msg: String):
        cmd = msg.data.strip()
        if cmd == "가자":
            self.state = "TRACKING"
            rospy.loginfo("[STATE] TRACKING 시작")
        elif cmd == "돌아가":
            self.state = "RETURN_SEQUENCE"
            self.return_phase = 0
            rospy.loginfo("[STATE] RETURN 시퀀스 시작 (회전→트래킹→회전)")

    def cb_err(self, msg: Float32):
        self.last_err = msg.data

    def cb_status(self, msg: String):
        if self.state == "TRACKING" and msg.data == "finish":
            rospy.loginfo("[STATE] 라인 종료 → 정지")
            self.stop()
            self.state = "IDLE"
        elif self.state == "RETURN_SEQUENCE":
            if self.return_phase == 1 and msg.data == "finish":
                rospy.loginfo("[STATE] 돌아가기 라인 종료 → 마지막 회전")
                self.rotate_180()
                self.stop()
                self.state = "IDLE"
                rospy.loginfo("[STATE] RETURN 완료")

    def control_loop(self, _):
        if self.state == "IDLE":
            return
        if self.state == "TRACKING":
            self.track_step()
        elif self.state == "RETURN_SEQUENCE":
            if self.return_phase == 0:
                self.rotate_180()
                self.return_phase = 1
                rospy.loginfo("[STATE] 반환 경로 라인트래킹 시작")
            elif self.return_phase == 1:
                self.track_step()
            # 마지막 회전은 cb_status에서 처리

    def track_step(self):
        twist = Twist()
        twist.linear.x = self.v_track
        twist.angular.z = -self.Kp * self.last_err
        self.pub_cmd.publish(twist)

    def rotate_180(self):
        # 좌회전 기준(양수 각속도). 필요시 방향 바꾸세요.
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.w_rotate
        t_end = time.time() + self.rotate_time_180
        rate = rospy.Rate(50)
        while time.time() < t_end and not rospy.is_shutdown():
            self.pub_cmd.publish(twist)
            rate.sleep()
        self.stop()

    def stop(self):
        self.pub_cmd.publish(Twist())

if __name__ == "__main__":
    rospy.init_node("state_node")
    StateNode()
    rospy.spin()

