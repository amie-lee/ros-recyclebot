#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Bool

def clamp(x, lo, hi): return lo if x < lo else (hi if x > hi else x)

class LineDetectorCurve:
    """
    라인(테이프) 추종:
      - 하부 ROI에서 가로 오차(cx) + 방향(헤딩) 오차로 조향
      - 헤딩 오차 클수록 자동 감속
      - 잠깐 놓치면 짧게 재탐색(스캔)
      - 작은 오차일수록 조향 약화(스케일링) + 조향 변화율 제한(슬루)
      - ★ 보강: center_shift_frac(가상 중앙 이동), turn_gain_left/right(좌우 비대칭 증폭)
    """

    def __init__(self):
        rospy.init_node("line_detector")

        # === 제어 파라미터 ===
        self.kp = rospy.get_param("~kp", 0.24)         # 가로 오차 비례
        self.kd = rospy.get_param("~kd", 0.06)         # 가로 오차 미분
        self.kh = rospy.get_param("~kh", 0.35)         # 헤딩 오차 비례
        self.ema_alpha = rospy.get_param("~ema_alpha", 0.35)
        self.err_deadband = rospy.get_param("~err_deadband", 0.03)
        self.steer_sign = rospy.get_param("~steer_sign", -1.0)  # 조향 부호(환경에 따라 ±1.0)

        # === 속도/제한 ===
        self.base_speed = rospy.get_param("~base_speed", 0.18)
        self.min_speed  = rospy.get_param("~min_speed", 0.10)
        self.max_omega  = rospy.get_param("~max_omega", 0.55)

        # (보강) 작은 오차일수록 조향 약화
        self.omega_scale_gamma = rospy.get_param("~omega_scale_gamma", 0.7)  # 0.5~1.0
        self.omega_scale_min   = rospy.get_param("~omega_scale_min", 0.35)   # 0~1
        # (보강) 조향 변화율 제한(슬루 레이트)
        self.omega_rate_limit  = rospy.get_param("~omega_rate_limit", 2.0)   # rad/s^2

        # === 시각 처리 ===
        self.bin_thresh = rospy.get_param("~bin_thresh", 110)
        self.invert = rospy.get_param("~invert", True)
        self.use_adaptive = rospy.get_param("~use_adaptive", False)
        self.adapt_block = rospy.get_param("~adapt_block", 31)
        self.adapt_C     = rospy.get_param("~adapt_C", 5)

        # 하부 ROI(근거리 중심)
        self.roi_y_start = rospy.get_param("~roi_y_start", 0.76)
        self.roi_height  = rospy.get_param("~roi_height", 0.18)
        # 보조 ROI(멀리보기) 기본 끔
        self.use_lookahead = rospy.get_param("~use_lookahead", False)
        self.lookahead_gap = rospy.get_param("~lookahead_gap", 0.18)

        self.min_area = rospy.get_param("~min_area", 300)
        self.min_white_pixels = rospy.get_param("~min_white_pixels", 300)

        # 라인 놓치면 짧게 스캔
        self.search_omega = rospy.get_param("~search_omega", 0.3)
        self.search_ms    = rospy.get_param("~search_ms", 600)

        # ====== ★ 추가 보강 파라미터 ======
        # 화면 폭 대비 중앙 이동 비율(음수면 중앙을 왼쪽으로 이동 → 좌회전 더 민감)
        self.center_shift_frac = rospy.get_param("~center_shift_frac", 0.0)   # 예: -0.04
        # 좌/우 회전 비대칭 증폭(좌회전이 약하면 left > 1.0)
        self.turn_gain_left  = rospy.get_param("~turn_gain_left", 1.00)       # 예: 1.15 ~ 1.25
        self.turn_gain_right = rospy.get_param("~turn_gain_right", 1.00)

        # 통신
        self.bridge = CvBridge()
        self.pub_cmd = rospy.Publisher("line_tracker/cmd_vel", Twist, queue_size=1)
        self.pub_panel = rospy.Publisher("/line_follower/panel", Image, queue_size=1)
        self.pub_overlay = rospy.Publisher("/line_follower/overlay", Image, queue_size=1)
        self.pub_found = rospy.Publisher("/line_follower/found", Bool, queue_size=1)
        rospy.Subscriber("image_raw", Image, self.cb, queue_size=1)  # 런치에서 /usb_cam/image_raw 로 리맵

        # 상태
        self.prev_time = None
        self.err_filt  = None
        self.prev_err  = 0.0
        self.prev_omega = 0.0
        self.last_found_ts = 0.0
        self.scan_dir = 1

        rospy.loginfo("line_detector_curve (reinforced + center/turn-gain) started")

    # --- 유틸 ---
    def _binarize(self, roi_gray):
        if self.use_adaptive:
            m = cv2.adaptiveThreshold(
                roi_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
                self.adapt_block | 1, self.adapt_C
            )
        else:
            _, m = cv2.threshold(roi_gray, self.bin_thresh, 255, cv2.THRESH_BINARY)
        if self.invert:
            m = cv2.bitwise_not(m)
        kernel = np.ones((5,5), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        return m

    def _heading_deg_from_contour(self, c):
        vx, vy, _, _ = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        ang = np.degrees(np.arctan2(abs(vx), abs(vy)))  # 0=수직, 90=수평
        return ang

    def _combine_heading(self, ang_main, ang_look):
        if ang_look is None: return ang_main
        return 0.7*ang_main + 0.3*ang_look

    def _publish_stop(self):
        self.pub_cmd.publish(Twist())

    # --- 콜백 ---
    def cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame  = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        h, w, _ = frame.shape

        y0 = int(self.roi_y_start * h)
        y1 = min(h, int((self.roi_y_start + self.roi_height) * h))
        roi = frame[y0:y1, :].copy()

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, y0), (w-1, y1-1), (0,255,255), 2)
        cv2.line(overlay, (w//2, y0), (w//2, y1-1), (255,0,0), 2)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = self._binarize(gray)
        nonzero = cv2.countNonZero(mask)

        moved = False
        ang_deg_main = None
        ang_deg_look = None

        if nonzero >= self.min_white_pixels:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(c) >= self.min_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])

                        # === 가로 오차 ===
                        # ★ 가상 중앙 이동 적용
                        center_x = (w / 2.0) * (1.0 + self.center_shift_frac)
                        err = (cx - center_x) / (w / 2.0)
                        if abs(err) < self.err_deadband:
                            err = 0.0

                        # === 헤딩 오차 ===
                        ang_deg_main = self._heading_deg_from_contour(c)
                        if self.use_lookahead:
                            ya0 = max(0, int((self.roi_y_start - self.lookahead_gap) * h))
                            ya1 = min(h, int((self.roi_y_start - self.lookahead_gap + self.roi_height) * h))
                            if ya1 - ya0 > 2:
                                roi2 = frame[ya0:ya1, :].copy()
                                mask2 = self._binarize(cv2.cvtColor(roi2, cv2.COLOR_BGR2GRAY))
                                cnts2,_ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                if cnts2:
                                    c2 = max(cnts2, key=cv2.contourArea)
                                    if cv2.contourArea(c2) >= self.min_area*0.5:
                                        ang_deg_look = self._heading_deg_from_contour(c2)
                                        rect2 = cv2.minAreaRect(c2)
                                        box2 = cv2.boxPoints(rect2); box2 = np.int0(box2)
                                        cv2.rectangle(overlay, (0, ya0), (w-1, ya1-1), (0,200,200), 2)
                                        rr2 = roi2.copy()
                                        cv2.drawContours(rr2, [box2], 0, (0,0,255), 2)
                                        overlay[ya0:ya1,:] = cv2.addWeighted(overlay[ya0:ya1,:], 0.5, rr2, 0.5, 0)

                        # === 제어 ===
                        now = rospy.get_time()
                        dt = (now - self.prev_time) if self.prev_time else 0.0
                        if self.err_filt is None:
                            self.err_filt = err
                        else:
                            self.err_filt = (1 - self.ema_alpha)*self.err_filt + self.ema_alpha*err
                        de = ((self.err_filt - self.prev_err)/dt) if dt > 0 else 0.0
                        self.prev_err  = self.err_filt
                        self.prev_time = now

                        ang_deg = self._combine_heading(ang_deg_main, ang_deg_look)
                        ang_rad = np.radians(ang_deg if ang_deg is not None else 0.0)

                        sign_e = 0.0 if self.err_filt == 0 else (1.0 if self.err_filt > 0 else -1.0)
                        ang_term = sign_e * ang_rad

                        # 원시 조향
                        base = (self.kp*self.err_filt + self.kd*de + self.kh*ang_term)
                        omega = self.steer_sign * base

                        # ★ 좌/우 비대칭 증폭 (ROS 규약: +z = 좌회전)
                        turn_dir = 1 if (self.steer_sign * base) > 0 else (-1 if (self.steer_sign * base) < 0 else 0)
                        if turn_dir > 0:      # 좌회전
                            omega *= float(self.turn_gain_left)
                        elif turn_dir < 0:    # 우회전
                            omega *= float(self.turn_gain_right)

                        omega = clamp(omega, -self.max_omega, self.max_omega)

                        # (보강1) 작은 오차일 때 조향 약화
                        scale = max(self.omega_scale_min,
                                    min(1.0, abs(self.err_filt) ** self.omega_scale_gamma))
                        omega *= scale

                        # (보강2) 조향 변화율 제한(슬루)
                        omega_raw = omega
                        if dt > 0:
                            max_step = self.omega_rate_limit * dt
                            delta = omega_raw - self.prev_omega
                            if   delta >  max_step: omega = self.prev_omega + max_step
                            elif delta < -max_step: omega = self.prev_omega - max_step
                            else:                   omega = omega_raw
                        self.prev_omega = omega

                        # 속도: 헤딩 크면 감속
                        slow = clamp(1.0 - (ang_rad/0.7), 0.0, 1.0)  # ~40°부터 감속
                        v = clamp(self.base_speed*(0.5 + 0.5*slow), self.min_speed, self.base_speed)

                        t = Twist(); t.linear.x = float(v); t.angular.z = float(omega)
                        self.pub_cmd.publish(t)
                        self.pub_found.publish(Bool(data=True))
                        moved = True
                        self.last_found_ts = now

                        # 시각화
                        rect = cv2.minAreaRect(c)
                        box  = cv2.boxPoints(rect); box = np.int0(box)
                        viz = roi.copy()
                        cv2.drawContours(viz, [box], 0, (0,0,255), 2)
                        cv2.line(viz, (cx,0), (cx, viz.shape[0]-1), (0,255,0), 2)
                        overlay[y0:y1,:] = cv2.addWeighted(overlay[y0:y1,:], 0.5, viz, 0.5, 0)
                        txt = f"pix={nonzero} e={self.err_filt:.3f} ang={ang_deg if ang_deg is not None else 0:.1f}° v={v:.2f} w={omega:.2f}"
                        cv2.putText(overlay, txt, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60,200,60), 2)

        if not moved:
            now = rospy.get_time()
            dt_ms = (now - self.last_found_ts)*1000.0
            t = Twist()
            if dt_ms < self.search_ms:
                t.angular.z = self.steer_sign * (self.search_omega * self.scan_dir)
                t.linear.x = 0.0
                self.pub_cmd.publish(t)
                self.pub_found.publish(Bool(data=False))
            else:
                self._publish_stop()
                self.pub_found.publish(Bool(data=False))
                self.scan_dir *= -1  # 다음엔 반대쪽 스캔

        # 패널
        H = 240
        def fit_h(img, H): return cv2.resize(img, (int(img.shape[1]*H/img.shape[0]), H))
        panel = cv2.hconcat([fit_h(frame,H), fit_h(overlay,H)])
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay,"bgr8"))
        self.pub_panel.publish(self.bridge.cv2_to_imgmsg(panel,"bgr8"))

if __name__ == "__main__":
    LineDetectorCurve()
    rospy.spin()
