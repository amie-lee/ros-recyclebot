#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LineDetectorStraight:
    """
    직선 테이프가 ROI 안에 '있기만' 하면 전진.
    - 중심(cx) 오차만 PD로 소폭 보정
    - 라인 놓치면 정지
    - 디버그: overlay / panel 퍼블리시
    """

    def __init__(self):
        rospy.init_node("line_detector_straight")

        # ===== 파라미터 =====
        self.kp = rospy.get_param("~kp", 0.30)
        self.kd = rospy.get_param("~kd", 0.05)
        self.base_speed = rospy.get_param("~base_speed", 0.22)

        self.bin_thresh = rospy.get_param("~bin_thresh", 110)
        self.invert = rospy.get_param("~invert", True)        # 어두운 선: True
        self.roi_y_start = rospy.get_param("~roi_y_start", 0.70)
        self.roi_height  = rospy.get_param("~roi_height", 0.22)

        self.min_area = rospy.get_param("~min_area", 500)
        self.min_white_pixels = rospy.get_param("~min_white_pixels", 800)
        self.stop_when_lost = rospy.get_param("~stop_when_lost", True)

        # 헌팅 완화
        self.max_omega = rospy.get_param("~max_omega", 0.6)   # 각속도 제한(라디안/초)
        self.ema_alpha = rospy.get_param("~ema_alpha", 0.35)  # 에러 평활
        self.err_deadband = rospy.get_param("~err_deadband", 0.02)  # 오차 소거

        # ===== 통신 =====
        self.bridge = CvBridge()
        self.pub_twist   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_overlay = rospy.Publisher("/line_follower/overlay", Image, queue_size=1)
        self.pub_panel   = rospy.Publisher("/line_follower/panel", Image, queue_size=1)

        # 카메라 구독 (런치에서 image_raw -> /usb_cam/image_raw 로 리맵)
        rospy.Subscriber("image_raw", Image, self.cb, queue_size=1)

        # PD 상태
        self.prev_error = 0.0
        self.prev_time = None
        self.err_filt = None

        rospy.loginfo("line_detector_straight (simple) started")

    def _publish_stop(self):
        if self.stop_when_lost:
            self.pub_twist.publish(Twist())

    def cb(self, msg: Image):
        # 1) 이미지 → BGR
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame  = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        h, w, _ = frame.shape

        # 2) ROI
        y0 = int(self.roi_y_start * h)
        y1 = min(h, int((self.roi_y_start + self.roi_height) * h))
        roi = frame[y0:y1, :].copy()

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, y0), (w-1, y1-1), (0, 255, 255), 2)
        cv2.line(overlay, (w//2, y0), (w//2, y1-1), (255, 0, 0), 2)  # 화면 중앙선

        # 3) 이진화
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.bin_thresh, 255, cv2.THRESH_BINARY)
        if self.invert:
            mask = cv2.bitwise_not(mask)
        # 4) 모폴로지
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 5) 라인 유무 + 중심 오차
        nonzero = cv2.countNonZero(mask)
        moved = False
        omega = 0.0

        if nonzero >= self.min_white_pixels:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) >= self.min_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        # -1..1 범위 오차
                        error = (cx - (w / 2.0)) / (w / 2.0)

                        # 데드밴드
                        if abs(error) < self.err_deadband:
                            error = 0.0

                        # EMA + PD + 제한
                        now = rospy.get_time()
                        dt = (now - self.prev_time) if self.prev_time else 0.0
                        if self.err_filt is None:
                            self.err_filt = error
                        else:
                            self.err_filt = (1 - self.ema_alpha) * self.err_filt + self.ema_alpha * error
                        de = ((self.err_filt - self.prev_error) / dt) if dt > 0 else 0.0
                        omega = -(self.kp * self.err_filt + self.kd * de)
                        omega = max(-self.max_omega, min(self.max_omega, omega))

                        # 속도 명령: 라인만 보이면 항상 전진
                        t = Twist()
                        t.linear.x = float(self.base_speed)
                        t.angular.z = float(omega)
                        rospy.loginfo_throttle(0.5, f"[move] lin={t.linear.x:.2f} ang={t.angular.z:.2f} err={self.err_filt:.3f} nonzero={nonzero}")
                        self.pub_twist.publish(t)
                        moved = True

                        # 디버그 오버레이
                        rect = cv2.minAreaRect(c)
                        box = np.int0(cv2.boxPoints(rect))
                        cv2.drawContours(roi, [box], 0, (0, 0, 255), 2)
                        cv2.line(roi, (cx, 0), (cx, roi.shape[0]-1), (0, 255, 0), 2)

                        self.prev_error = self.err_filt
                        self.prev_time = now

        if not moved:
            rospy.logwarn_throttle(0.5, "[stop] no line/conditions → zero Twist")
            self._publish_stop()

        # 6) 패널/오버레이 퍼블리시
        overlay[y0:y1, :] = cv2.addWeighted(overlay[y0:y1, :], 0.5, roi, 0.5, 0)
        H = 240
        def fit_h(img, H):
            return cv2.resize(img, (int(img.shape[1]*H/img.shape[0]), H))
        panel = cv2.hconcat([fit_h(frame, H), fit_h(overlay, H)])

        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
        self.pub_panel.publish(self.bridge.cv2_to_imgmsg(panel, "bgr8"))

if __name__ == "__main__":
    LineDetectorStraight()
    rospy.spin()
