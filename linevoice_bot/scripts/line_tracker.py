#!/usr/bin/env python3
import time
import rospy
import pigpio
from geometry_msgs.msg import Twist
from std_msgs.msg import String

STATE_IDLE = 0
STATE_ROTATE = 1
STATE_TRACK = 2
STATE_ROTATE_BACK = 3

class LineTracker:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio daemon not running.")
            rospy.signal_shutdown("pigpio not connected")
            return

        # params
        pins = rospy.get_param("~line_sensor")
        self.pin_left = int(pins["left"])
        self.pin_center = int(pins["center"])
        self.pin_right = int(pins["right"])
        self.active_low = bool(pins.get("active_low", True))
        self.pull_up = bool(pins.get("pull_up", True))

        drive = rospy.get_param("~drive")
        self.base_linear = float(drive.get("base_linear", 0.4))
        self.turn_gain   = float(drive.get("turn_gain", 0.6))
        self.clamp       = float(drive.get("clamp", 1.0))

        turning = rospy.get_param("~turning")
        self.turn_speed   = float(turning.get("turn_speed", 0.6))
        self.rotate_secs  = float(turning.get("rotate_secs", 1.2))

        le = rospy.get_param("~line_end")
        self.lost_thresh  = int(le.get("lost_count_thresh", 10))
        self.rate_hz      = int(le.get("rate_hz", 50))

        # GPIO setup
        for p in (self.pin_left, self.pin_center, self.pin_right):
            self.pi.set_mode(p, pigpio.INPUT)
            if self.pull_up:
                self.pi.set_pull_up_down(p, pigpio.PUD_UP)
            else:
                self.pi.set_pull_up_down(p, pigpio.PUD_OFF)

        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub_state = rospy.Publisher("tracking_state", String, queue_size=10, latch=True)
        rospy.Subscriber("voice_cmd", String, self.cb_voice, queue_size=5)

        self.state = STATE_IDLE
        self.timer_rotate_end = 0.0
        self.return_mode = False  # '돌아가' 동작 플래그

        self.pub_state.publish(String("IDLE"))

    def read_sensor(self):
        def val(pin):
            v = self.pi.read(pin)
            return 1 - v if self.active_low else v
        return val(self.pin_left), val(self.pin_center), val(self.pin_right)

    def cb_voice(self, msg):
        data = msg.data.upper()
        if data == "GO":
            self.return_mode = False
            self.start_sequence()
        elif data == "RETURN":
            self.return_mode = True
            self.start_sequence()

    def start_sequence(self):
        # 먼저 제자리 회전 여부
        if self.return_mode:
            self.state = STATE_ROTATE
            self.timer_rotate_end = time.time() + self.rotate_secs
            self.pub_state.publish(String("ROTATE_TO_RETURN"))
        else:
            self.state = STATE_TRACK
            self.pub_state.publish(String("TRACK_FORWARD"))

    def do_rotate(self, clockwise=True):
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = self.turn_speed if clockwise else -self.turn_speed
        self.pub_cmd.publish(tw)

        if time.time() >= self.timer_rotate_end:
            self.state = STATE_TRACK
            self.pub_state.publish(String("TRACK_RETURN" if self.return_mode else "TRACK_FORWARD"))

    def do_rotate_back(self):
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = -self.turn_speed  # 반대 방향 회전
        self.pub_cmd.publish(tw)
        if time.time() >= self.timer_rotate_end:
            self.stop_robot()
            self.state = STATE_IDLE
            self.pub_state.publish(String("IDLE"))

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        lost_count = 0

        while not rospy.is_shutdown():
            if self.state == STATE_IDLE:
                self.stop_robot()
                rate.sleep()
                continue

            if self.state == STATE_ROTATE:
                self.do_rotate(clockwise=True)
                rate.sleep()
                continue

            if self.state == STATE_ROTATE_BACK:
                self.do_rotate_back()
                rate.sleep()
                continue

            # STATE_TRACK
            L, C, R = self.read_sensor()
            on_line = (L + C + R) > 0

            if not on_line:
                lost_count += 1
            else:
                lost_count = 0

            # 라인 종료 판단
            if lost_count >= self.lost_thresh:
                self.stop_robot()
                if self.return_mode:
                    # 도착 후 다시 반대 방향으로 회전
                    self.state = STATE_ROTATE_BACK
                    self.timer_rotate_end = time.time() + self.rotate_secs
                    self.pub_state.publish(String("ROTATE_BACK_AT_END"))
                else:
                    self.state = STATE_IDLE
                    self.pub_state.publish(String("IDLE"))
                rate.sleep()
                continue

            # 단순 P-제어: 좌:-1, 중:0, 우:+1 가중 평균
            err = (-1 if L else 0) + (0 if C else 0) + (1 if R else 0)
            # C의 존재는 직진 유지에 우호적이므로 L/R가 동시에 켜질 때만 에러 반영 강화
            if C and not (L and R):
                err = 0

            tw = Twist()
            tw.linear.x = max(0.0, min(self.clamp, self.base_linear))
            tw.angular.z = max(-self.clamp, min(self.clamp, self.turn_gain * err))
            self.pub_cmd.publish(tw)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("line_tracker")
    node = LineTracker()
    node.run()

