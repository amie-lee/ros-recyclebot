#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time, threading
from std_msgs.msg import String
from geometry_msgs.msg import Twist

"""
voice_cmd_gate_ext.py
- /voice_cmd (std_msgs/String: 'start'|'stop'|'turn_around')를 구독해 주행 게이트 제어
- /line_tracker/cmd_vel 입력을 받아, 게이트 ON일 때만 /cmd_vel로 전달
- 'turn_around' 시 지정한 시간/각속도로 제자리 회전 수행
- 'stop' 수신 시 어떤 상태든 즉시 정지(회전 중단 포함)

ROS params:
~cmd_topic_in      (str, default: "/line_tracker/cmd_vel")
~cmd_topic_out     (str, default: "/cmd_vel")
~voice_cmd_topic   (str, default: "/voice_cmd")
~turn180_dir       (str, default: "left")  # left|right
~turn180_omega     (float, default: 0.7)   # rad/s
~turn180_duration  (float, default: 7.5)   # s
"""

class Gate(object):
    def __init__(self):
        # ----- 상태/락/이벤트를 가장 먼저 초기화 (콜백 안전) -----
        self.lock = threading.Lock()
        self.on = False
        self.abort_turn = threading.Event()  # <-- 누락되어 크래시 나던 부분
        self.turn_thread = None

        # ----- 파라미터 -----
        self.in_topic   = rospy.get_param("~cmd_topic_in",  "/line_tracker/cmd_vel")
        self.out_topic  = rospy.get_param("~cmd_topic_out", "/cmd_vel")
        self.vcmd_topic = rospy.get_param("~voice_cmd_topic", "/voice_cmd")
        self.turn_dir   = rospy.get_param("~turn180_dir", "left").lower()
        self.turn_omega = float(rospy.get_param("~turn180_omega", 0.7))
        self.turn_dur   = float(rospy.get_param("~turn180_duration", 7.5))

        # ----- 퍼블리셔/서브스크라이버 -----
        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.in_topic,  Twist,  self.cb_twist, queue_size=50)
        rospy.Subscriber(self.vcmd_topic, String, self.cb_voice, queue_size=10)

        rospy.loginfo("voice_cmd_gate_ext: in=%s out=%s voice=%s",
                      self.in_topic, self.out_topic, self.vcmd_topic)

    # 주행 명령(라인 추종) 입력 -> 게이트 ON이면 통과
    def cb_twist(self, msg: Twist):
        with self.lock:
            gate = self.on
        if gate:
            self.pub.publish(msg)

    # 음성 명령
    def cb_voice(self, msg: String):
        s = (msg.data or "").strip().lower().replace(" ", "")
        if s == "start":
            with self.lock:
                self.on = True
            rospy.loginfo("[gate] START -> enabled")

        elif s == "stop":
            # 즉시 정지: 게이트 끄고 회전 중단, 정지 토픽 발행
            with self.lock:
                self.on = False
            try:
                self.abort_turn.set()
            except Exception:
                # 혹시라도 속성 초기화 경쟁이 있었다면 방어적으로 무시
                pass
            self.pub.publish(Twist())
            rospy.loginfo("[gate] STOP -> disabled (brake)")

        elif s == "turn_around":
            rospy.loginfo("[gate] TURN AROUND requested")
            # 회전은 스레드로 실행, 중복 방지
            if self.turn_thread and self.turn_thread.is_alive():
                rospy.loginfo("[gate] turn already running; ignoring")
                return
            self.abort_turn.clear()
            self.turn_thread = threading.Thread(target=self._do_turn, daemon=True)
            self.turn_thread.start()

    # 제자리 회전 루틴(중단 이벤트 감시)
    def _do_turn(self):
        with self.lock:
            was_on = self.on
            self.on = False  # 회전 동안 라인 추종 차단

        twist = Twist()
        twist.angular.z = +self.turn_omega if self.turn_dir == "left" else -self.turn_omega
        t0 = time.time()
        rate = rospy.Rate(30)

        rospy.loginfo("[gate] turning: dir=%s, omega=%.3f, dur=%.2fs",
                      self.turn_dir, self.turn_omega, self.turn_dur)

        while not rospy.is_shutdown() and (time.time() - t0) < self.turn_dur:
            if self.abort_turn.is_set():
                rospy.loginfo("[gate] turn aborted by STOP")
                break
            self.pub.publish(twist)
            rate.sleep()

        # 정지 한번 내보내고, 원래 게이트 상태 복원
        self.pub.publish(Twist())
        with self.lock:
            self.on = False

def main():
    rospy.init_node("voice_cmd_gate_ext")
    Gate()
    rospy.spin()

if __name__ == "__main__":
    main()
