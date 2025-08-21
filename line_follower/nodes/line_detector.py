#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class LineDetector:
    def __init__(self):
        rospy.init_node("line_detector")

        # ===== 파라미터 =====
        self.kp = rospy.get_param("~kp", 0.8)
        self.kd = rospy.get_param("~kd", 0.02)
        self.base_speed = rospy.get_param("~base_speed", 0.28)

        self.bin_thresh = rospy.get_param("~bin_thresh", 120)  # 이진화 임계값
        self.invert = rospy.get_param("~invert", True)         # 어두운 선이면 True
        self.roi_y_start = rospy.get_param("~roi_y_start", 0.6)
        self.roi_height = rospy.get_param("~roi_height", 0.3)

        # 컨투어 최소 면적(너무 작은 노이즈 무시)
        self.min_area = rospy.get_param("~min_area", 500)

        # ★ 라인이 있다고 인정할 최소 흰 픽셀 수 (라인이 끊기면 이 값 미만 → 정지)
        self.min_white_pixels = rospy.get_param("~min_white_pixels", 1200)

        # 라인 없을 때 정지 명령 보낼지
        self.stop_when_lost = rospy.get_param("~stop_when_lost", True)

        # ===== 통신 =====
        self.bridge = CvBridge()
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_debug = rospy.Publisher("/line_follower/debug", Image, queue_size=1)
        rospy.Subscriber("image_raw", Image, self.cb, queue_size=1)

        # PD 상태
        self.prev_error = 0.0
        self.prev_time = None

        rospy.loginfo("line_detector started")

    def _publish_stop(self):
        if self.stop_when_lost:
            self.pub_twist.publish(Twist())  # linear=0, angular=0

    def cb(self, msg: Image):
        # 1) rgb8 → BGR 변환 (usb_cam이 rgb8을 내보내는 환경)
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

        h, w, _ = frame.shape
        y0 = int(self.roi_y_start * h)
        y1 = min(h, int((self.roi_y_start + self.roi_height) * h))
        roi = frame[y0:y1, :]

        # 2) 그레이 & 이진화
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.bin_thresh, 255, cv2.THRESH_BINARY)
        if self.invert:
            mask = cv2.bitwise_not(mask)

        # 3) 모폴로지로 노이즈 정리
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 4) ★ 흰 픽셀 수로 “라인 유무” 1차 판단
        nonzero = cv2.countNonZero(mask)
        found = False
        cx = None

        if nonzero >= self.min_white_pixels:
            # 5) 컨투어 중 가장 큰 것만 사용
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) >= self.min_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        error = (cx - (w / 2.0)) / (w / 2.0)

                        # PD 제어
                        now = rospy.get_time()
                        dt = (now - self.prev_time) if self.prev_time else 0.0
                        de = ((error - self.prev_error) / dt) if dt > 0 else 0.0
                        omega = -(self.kp * error + self.kd * de)

                        # 속도 명령
                        t = Twist()
                        t.linear.x = float(self.base_speed)
                        t.angular.z = float(omega)
                        self.pub_twist.publish(t)

                        self.prev_error = error
                        self.prev_time = now
                        found = True

        if not found:
            # 라인 없을 때 정지
            self._publish_stop()

        # 6) 디버그 이미지는 항상 퍼블리시
        dbg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # 중앙 기준선(파랑)
        cv2.line(dbg, (w // 2, 0), (w // 2, dbg.shape[0] - 1), (255, 0, 0), 2)
        # 검출된 중심(초록)
        if found and cx is not None:
            cv2.line(dbg, (cx, 0), (cx, dbg.shape[0] - 1), (0, 255, 0), 2)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))


if __name__ == "__main__":
    LineDetector()
    rospy.spin()

