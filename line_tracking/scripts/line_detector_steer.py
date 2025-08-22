#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LineDetectorSteer:
    """
    라인이 ROI에 보이면 전진하며, 중심 오차만큼 좌/우로 살짝 회전:
      - 라인이 화면 중앙보다 오른쪽이면 → 우회전(오른쪽으로 살짝 틀기)
      - 라인이 화면 중앙보다 왼쪽이면 → 좌회전
    라인 놓치면 정지.
    """

    def __init__(self):
        rospy.init_node("line_detector")

        # ===== 파라미터 =====
        self.kp = rospy.get_param("~kp", 0.28)           # 비례 이득 (크면 민감)
        self.kd = rospy.get_param("~kd", 0.05)           # 미분 이득 (진동 억제)
        self.base_speed = rospy.get_param("~base_speed", 0.22)

        self.bin_thresh = rospy.get_param("~bin_thresh", 110)
        self.invert = rospy.get_param("~invert", True)   # 어두운 선이면 True
        self.roi_y_start = rospy.get_param("~roi_y_start", 0.70)
        self.roi_height  = rospy.get_param("~roi_height", 0.22)

        self.min_area = rospy.get_param("~min_area", 300)
        self.min_white_pixels = rospy.get_param("~min_white_pixels", 300)

        self.err_deadband = rospy.get_param("~err_deadband", 0.02)  # 아주 작은 오차 무시
        self.max_omega    = rospy.get_param("~max_omega", 0.6)      # 각속도 제한(rad/s)
        self.ema_alpha    = rospy.get_param("~ema_alpha", 0.35)     # 에러 평활(0~1)
        # 조향 부호(필요시 1.0으로 바꿔 반전):  omega = steer_sign*(kp*e + kd*de)
        self.steer_sign   = rospy.get_param("~steer_sign", -1.0)

        self.stop_when_lost = rospy.get_param("~stop_when_lost", True)

        # ===== 통신 =====
        self.bridge = CvBridge()
        self.pub_cmd   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_panel = rospy.Publisher("/line_follower/panel", Image, queue_size=1)
        self.pub_overlay = rospy.Publisher("/line_follower/overlay", Image, queue_size=1)
        # 런치에서 image_raw -> /usb_cam/image_raw 로 리맵해두기
        rospy.Subscriber("image_raw", Image, self.cb, queue_size=1)

        # 상태
        self.prev_time = None
        self.prev_err  = 0.0
        self.err_filt  = None

        rospy.loginfo("line_detector_steer started")

    def _stop(self):
        if self.stop_when_lost:
            self.pub_cmd.publish(Twist())

    def cb(self, msg: Image):
        # 1) 이미지
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame  = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        h, w, _ = frame.shape

        # 2) ROI
        y0 = int(self.roi_y_start * h)
        y1 = min(h, int((self.roi_y_start + self.roi_height) * h))
        roi = frame[y0:y1, :].copy()

        # 3) 이진화
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.bin_thresh, 255, cv2.THRESH_BINARY)
        if self.invert:
            mask = cv2.bitwise_not(mask)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 4) 라인 유무 판단 및 중심(cx)
        nonzero = cv2.countNonZero(mask)
        moved = False
        omega = 0.0

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, y0), (w-1, y1-1), (0,255,255), 2)
        cv2.line(overlay, (w//2, y0), (w//2, y1-1), (255,0,0), 2)  # 화면 중앙선

        if nonzero >= self.min_white_pixels:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(c) >= self.min_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])      # ROI 좌표
                        # 중앙 기준 오차: (cx - w/2)/ (w/2)  → [-1,1]
                        error = (cx - (w/2.0)) / (w/2.0)

                        # 데드밴드 + 평활
                        if abs(error) < self.err_deadband:
                            error = 0.0
                        now = rospy.get_time()
                        dt = (now - self.prev_time) if self.prev_time else 0.0
                        if self.err_filt is None:
                            self.err_filt = error
                        else:
                            self.err_filt = (1 - self.ema_alpha)*self.err_filt + self.ema_alpha*error
                        de = ((self.err_filt - self.prev_err)/dt) if dt > 0 else 0.0

                        # 조향 계산: 라인이 오른쪽( error>0 )이면 omega<0(우회전) ⇒ steer_sign=-1.0이 기본
                        omega = self.steer_sign * (self.kp*self.err_filt + self.kd*de)
                        omega = max(-self.max_omega, min(self.max_omega, omega))

                        # 전진 + 조향
                        t = Twist()
                        t.linear.x  = float(self.base_speed)
                        t.angular.z = float(omega)
                        self.pub_cmd.publish(t)
                        moved = True

                        # 디버그 표시(ROI 상자, 중심선, 컨투어 박스)
                        rect = cv2.minAreaRect(c)
                        box  = cv2.boxPoints(rect); box = np.int0(box)
                        cv2.drawContours(roi, [box], 0, (0,0,255), 2)
                        cv2.line(roi, (cx, 0), (cx, roi.shape[0]-1), (0,255,0), 2)
                        info = f"nonzero={nonzero} err={self.err_filt:.3f} omega={omega:.2f}"
                        cv2.putText(overlay, info, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50,200,50), 2)

                        self.prev_err  = self.err_filt
                        self.prev_time = now

        if not moved:
            self._stop()

        # 패널
        overlay[y0:y1, :] = cv2.addWeighted(overlay[y0:y1, :], 0.5, roi, 0.5, 0)
        H = 240
        def fit_h(img, H): return cv2.resize(img, (int(img.shape[1]*H/img.shape[0]), H))
        panel = cv2.hconcat([fit_h(frame,H), fit_h(overlay,H)])
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
        self.pub_panel.publish(self.bridge.cv2_to_imgmsg(panel, "bgr8"))

if __name__ == "__main__":
    LineDetectorSteer()
    rospy.spin()
