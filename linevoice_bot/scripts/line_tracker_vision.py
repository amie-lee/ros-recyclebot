#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

STATE_IDLE = 0
STATE_ROTATE = 1
STATE_TRACK = 2
STATE_ROTATE_BACK = 3


class VisionLineTracker:
    def __init__(self):
        self.bridge = CvBridge()

        # ---- 상태 변수 ----
        self.state = STATE_IDLE
        self.return_mode = False
        self.timer_rotate_end = 0.0
        self.lost_count = 0
        self.first_frame_logged = False
        self.last_image_time = 0.0
        self.latest = None

        # ---- 주행/회전/라인종료 파라미터 ----
        drive   = rospy.get_param("~drive", {})
        turning = rospy.get_param("~turning", {})
        le      = rospy.get_param("~line_end", {})

        self.base_linear = float(drive.get("base_linear", 0.20))
        self.clamp       = float(drive.get("clamp", 1.0))

        self.turn_speed  = float(turning.get("turn_speed", 0.6))
        self.rotate_secs = float(turning.get("rotate_secs", 1.2))

        self.rate_hz     = int(le.get("rate_hz", 50))
        self.lost_thresh = int(le.get("lost_count_thresh", 10))

        # ---- 카메라/비전 파라미터 ----
        cam = rospy.get_param("~camera", {})

        # 토픽
        self.image_topic = cam.get("image_topic", "/usb_cam/image_raw")
        self.debug_topic = cam.get("debug_image_topic", "/line_debug")
        self.frame_id    = cam.get("frame_id", "camera")

        # ROI/리사이즈
        self.resize_w    = int(cam.get("resize_width", 320))
        self.roi_y       = int(cam.get("roi_y_from_bottom", 30))
        self.roi_h       = int(cam.get("roi_height", 150))

        # ----- 전처리/이진화 (Black-hat 기반) -----
        # 주의: black-hat은 "어두운 띠"를 밝게 만듦 → invert=False 권장
        self.clahe           = bool(cam.get("clahe", True))
        self.blur_kernel     = int(cam.get("blur_kernel", 5))
        self.use_blackhat    = bool(cam.get("use_blackhat", True))
        self.blackhat_kernel = int(cam.get("blackhat_kernel", 15))
        self.bh_threshold    = int(cam.get("bh_threshold", 25))
        # 아래 adaptive/otsu는 비활성(필요 시 전환 가능)
        self.use_adaptive    = bool(cam.get("use_adaptive", False))
        self.use_otsu        = bool(cam.get("use_otsu", False))
        self.binary_th       = int(cam.get("binary_threshold", 120))
        self.invert          = bool(cam.get("invert", False))  # black-hat 사용 시 False 권장

        # 조명 보정(이번 파이프라인에서는 기본 0 권장)
        self.illum_sigma     = int(cam.get("illum_sigma", 0))
        self.illum_gain      = float(cam.get("illum_gain", 0.0))
        self.illum_bias      = int(cam.get("illum_bias", 0))

        # 모폴로지/채움/노이즈 제거
        self.close_kernel    = int(cam.get("close_kernel", 9))
        self.open_kernel     = int(cam.get("open_kernel", 5))
        self.fill_holes      = bool(cam.get("fill_holes", True))
        self.noise_min_area  = int(cam.get("noise_min_area", 400))

        # 스캔/검출
        self.scan_rows   = int(cam.get("scan_rows", 6))
        self.min_pixels  = int(cam.get("min_pixels", 18))

        # (옵션) 블롭 필터
        self.use_blob_filter = bool(cam.get("use_blob_filter", True))
        self.blob_min_area   = int(cam.get("blob_min_area", 800))
        self.blob_max_width  = float(cam.get("blob_max_width", 0.55))  # ROI 너비 비율

        # ----- 제어 파라미터 -----
        # (오른쪽으로 치우치면 err>0, 우회전(z<0)이 나오도록 run()에서 부호 반전)
        self.kp         = float(cam.get("kp", -0.012))
        self.kd         = float(cam.get("kd", 0.12))
        self.ka         = float(cam.get("ka", -0.20))
        self.max_steer  = float(cam.get("max_steer", 0.45))
        self.steer_slew = float(cam.get("steer_slew", 2.5))

        # 보정/감속
        self.deadband_px   = float(cam.get("deadband_px", 6.0))
        self.ema_err_alpha = float(cam.get("ema_err_alpha", 0.25))
        self.speed_min     = float(cam.get("speed_min", 0.08))
        self.speed_gain    = float(cam.get("speed_gain", 0.8))

        # 필터 메모리
        self.err_hist = deque(maxlen=6)
        self.ang_hist = deque(maxlen=6)
        self.err_ema  = 0.0
        self.prev_err = 0.0
        self.prev_w   = 0.0
        self.prev_t   = rospy.get_time()
        self.prev_xref = None

        # ---- 퍼블/서브 ----
        self.pub_cmd   = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub_state = rospy.Publisher("tracking_state", String, queue_size=10, latch=True)
        self.pub_dbg   = rospy.Publisher(self.debug_topic, Image, queue_size=1)

        rospy.Subscriber("voice_cmd", String, self.cb_voice, queue_size=5)
        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)

        self.pub_state.publish(String("IDLE"))
        rospy.loginfo("line_tracker_vision: waiting images on %s", self.image_topic)

    # ------------------------- 콜백 -------------------------

    def cb_voice(self, msg: String):
        data = msg.data.upper().strip()
        if data == "GO":
            self.return_mode = False
            self.start_sequence()
        elif data == "RETURN":
            self.return_mode = True
            self.start_sequence()

    def cb_image(self, img_msg: Image):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            self.last_image_time = rospy.get_time()
            if not self.first_frame_logged:
                rospy.loginfo("line_tracker_vision: first image received on %s", self.image_topic)
                self.first_frame_logged = True
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)

    def start_sequence(self):
        if self.return_mode:
            self.state = STATE_ROTATE
            self.timer_rotate_end = time.time() + self.rotate_secs
            self.pub_state.publish(String("ROTATE_TO_RETURN"))
        else:
            self.state = STATE_TRACK
            self.pub_state.publish(String("TRACK_FORWARD"))

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

    def publish_debug(self, frame_bgr):
        if self.pub_dbg.get_num_connections() == 0:
            return
        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8"))
        except Exception as e:
            rospy.logwarn("debug pub: %s", e)

    # ------------------------- 비전 처리 -------------------------

    def _preprocess_binary(self, roi_bgr):
        """배경=검정, 테이프 영역=통짜 흰색으로 만드는 전처리.
           Black-hat → 고정 임계 → 클로징/채움 → 오프닝/블롭필터
        """
        # 0) L 채널 기반 그레이스케일 + 대비/블러
        lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
        L = lab[:, :, 0].astype(np.uint8)

        if self.clahe:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            L = clahe.apply(L)

        k = int(self.blur_kernel) | 1
        L = cv2.GaussianBlur(L, (k, k), 0)

        # (옵션) 조명 보정: 본 파이프라인에서는 보통 0 유지
        if self.illum_gain > 0 and self.illum_sigma >= 3:
            sigma = int(self.illum_sigma)
            illum = cv2.GaussianBlur(L, (0, 0), sigmaX=sigma, sigmaY=sigma)
            L = cv2.addWeighted(L, 1.0, illum, -self.illum_gain, self.illum_bias)
            L = np.clip(L, 0, 255).astype(np.uint8)

        # 1) Black-hat: 주변보다 어두운 띠(테이프)를 밝게
        if self.use_blackhat:
            bk = int(self.blackhat_kernel) | 1
            se = cv2.getStructuringElement(cv2.MORPH_RECT, (bk, bk))
            feat = cv2.morphologyEx(L, cv2.MORPH_BLACKHAT, se)
        else:
            feat = L

        # 2) 이진화 (feat가 클수록 테이프) → 흰색
        _, bw = cv2.threshold(feat, int(self.bh_threshold), 255, cv2.THRESH_BINARY)

        # 3) 클로징(내부 메움) + 4) 구멍 메우기
        ck = int(self.close_kernel) | 1
        se_close = cv2.getStructuringElement(cv2.MORPH_RECT, (ck, ck))
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, se_close, iterations=2)

        if self.fill_holes:
            h, w = bw.shape
            flood = bw.copy()
            mask = np.zeros((h + 2, w + 2), np.uint8)
            cv2.floodFill(flood, mask, (0, 0), 255)
            holes = cv2.bitwise_not(flood)
            bw = cv2.bitwise_or(bw, holes)

        # 5) 점 노이즈 제거
        ok = int(self.open_kernel) | 1
        se_open = cv2.getStructuringElement(cv2.MORPH_RECT, (ok, ok))
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, se_open, iterations=1)

        # (옵션) 면적 기반 블롭 필터는 process_frame에서 적용

        return bw

    def _keep_best_blob(self, bw, W):
        """가장 그럴듯한 세로 블롭만 유지(선택)."""
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(bw, connectivity=8)
        if num <= 1:
            return bw, None
        best_i, best_score, best_cx = None, -1e9, None
        for i in range(1, num):
            x, y, w, h, area = stats[i]
            if area < self.blob_min_area:
                continue
            if w > self.blob_max_width * W:
                continue
            cx = centroids[i][0]
            score = float(area)
            if self.prev_xref is not None:
                score -= abs(cx - self.prev_xref) * 2.0
            if score > best_score:
                best_score, best_i, best_cx = score, i, cx
        if best_i is None:
            return bw, None
        mask = (labels == best_i).astype(np.uint8) * 255
        return mask, float(best_cx)

    def process_frame(self, frame_bgr):
        """ok, on_line, (err_px, angle), debug_bgr"""
        h, w = frame_bgr.shape[:2]
        scale = float(self.resize_w) / float(w)
        img = cv2.resize(frame_bgr, (self.resize_w, int(h * scale)))
        H, W = img.shape[:2]

        # ROI: 하단 밴드
        y1 = max(0, H - self.roi_y - self.roi_h)
        y2 = max(0, H - self.roi_y)
        roi = img[y1:y2, :]

        bw = self._preprocess_binary(roi)

        cx_hint = None
        if self.use_blob_filter:
            bw, cx_hint = self._keep_best_blob(bw, W)

        # 여러 줄 스캔
        rows = max(3, int(self.scan_rows))
        ys = np.linspace(10, bw.shape[0] - 10, rows).astype(int)
        pts = []
        for y in ys:
            xs = np.where(bw[y, :] > 0)[0]
            if xs.size < self.min_pixels:
                continue
            if cx_hint is not None:
                lo = int(max(0, cx_hint - 80))
                hi = int(min(bw.shape[1] - 1, cx_hint + 80))
                xs = xs[(xs >= lo) & (xs <= hi)]
                if xs.size < self.min_pixels:
                    continue
            lo_p, hi_p = np.percentile(xs, [20, 80])
            xs = xs[(xs >= lo_p) & (xs <= hi_p)]
            if xs.size < self.min_pixels:
                continue
            cx = int(xs.mean())
            pts.append((cx, y))

        dbg = img.copy()
        cv2.rectangle(dbg, (0, y1), (W - 1, y2 - 1), (0, 255, 0), 2)

        if len(pts) < max(2, rows // 2):
            cv2.putText(dbg, "NO LINE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            dbg_bw = cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)
            dbg_bw = cv2.resize(dbg_bw, (dbg.shape[1], dbg.shape[0]))
            return False, False, 0, np.hstack([dbg_bw, dbg])

        pts = np.array(pts, dtype=np.float32)
        # 회귀: x = a*y + b
        A = np.vstack([pts[:, 1], np.ones(len(pts))]).T
        a, b = np.linalg.lstsq(A, pts[:, 0], rcond=None)[0]

        y_ref = pts[:, 1].max()
        x_ref = a * y_ref + b
        err_px = float(x_ref - (W / 2.0))
        angle  = float(np.arctan(a))
        self.prev_xref = x_ref

        # 디버그 오버레이
        x0 = int(a * 0 + b)
        x1 = int(a * (roi.shape[0] - 1) + b)
        cv2.line(dbg, (x0, y1), (x1, y2 - 1), (255, 0, 0), 2)
        for (x, y) in pts.astype(int):
            cv2.circle(dbg, (x, y + y1), 3, (0, 165, 255), -1)
        cv2.line(dbg, (W // 2, y1), (W // 2, y2 - 1), (255, 255, 255), 1)
        cv2.circle(dbg, (int(x_ref), int(y_ref) + y1), 6, (0, 165, 255), -1)
        cv2.putText(dbg, f"err={int(err_px)}px  ang={angle:.2f}rad",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        dbg_bw = cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)
        dbg_bw = cv2.resize(dbg_bw, (dbg.shape[1], dbg.shape[0]))
        dbg_out = np.hstack([dbg_bw, dbg])

        return True, True, (err_px, angle), dbg_out

    # ------------------------- 상태/제어 루프 -------------------------

    def do_rotate(self, clockwise=True):
        tw = Twist()
        tw.angular.z = self.turn_speed if clockwise else -self.turn_speed
        self.pub_cmd.publish(tw)
        if time.time() >= self.timer_rotate_end:
            self.state = STATE_TRACK
            self.pub_state.publish(String("TRACK_RETURN" if self.return_mode else "TRACK_FORWARD"))

    def do_rotate_back(self):
        tw = Twist()
        tw.angular.z = -self.turn_speed
        self.pub_cmd.publish(tw)
        if time.time() >= self.timer_rotate_end:
            self.stop_robot()
            self.state = STATE_IDLE
            self.pub_state.publish(String("IDLE"))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.state == STATE_IDLE:
                self.stop_robot()
                if self.latest is not None:
                    ok, on_line, res, dbg = self.process_frame(self.latest)
                    self.publish_debug(dbg)
                rate.sleep(); continue

            if self.state == STATE_ROTATE:
                self.do_rotate(clockwise=True); rate.sleep(); continue
            if self.state == STATE_ROTATE_BACK:
                self.do_rotate_back(); rate.sleep(); continue

            if self.latest is None:
                if rospy.get_time() - self.last_image_time > 2.0:
                    rospy.logwarn_throttle(2.0, "Waiting for images on %s ...", self.image_topic)
                self.stop_robot(); rate.sleep(); continue

            ok, on_line, res, dbg = self.process_frame(self.latest)
            self.publish_debug(dbg)

            if not ok or not on_line:
                self.lost_count += 1
            else:
                self.lost_count = 0

            if self.lost_count >= self.lost_thresh:
                self.stop_robot()
                if self.return_mode:
                    self.state = STATE_ROTATE_BACK
                    self.timer_rotate_end = time.time() + self.rotate_secs
                    self.pub_state.publish(String("ROTATE_BACK_AT_END"))
                else:
                    self.state = STATE_IDLE
                    self.pub_state.publish(String("IDLE"))
                rate.sleep(); continue

            # --- 제어 (PD + 각도 + 중앙값/EMA + 데드밴드 + 속도감속 + Slew) ---
            err_px, angle = res

            self.err_hist.append(err_px)
            self.ang_hist.append(angle)
            err_med = float(np.median(self.err_hist))
            ang_med = float(np.median(self.ang_hist))

            self.err_ema = (1.0 - self.ema_err_alpha) * self.err_ema + self.ema_err_alpha * err_med
            err_used = self.err_ema

            if abs(err_used) < self.deadband_px:
                err_ctrl = 0.0
            else:
                err_ctrl = err_used

            now = rospy.get_time()
            dt  = max(1e-3, now - self.prev_t)
            derr = (err_ctrl - self.prev_err) / dt
            derr = np.tanh(derr / 50.0) * 50.0

            # 부호 반전(우측=+err → 우회전 z<0)
            w_des = -(self.kp * err_ctrl + self.kd * derr + self.ka * ang_med)

            # 코너 감속
            norm_err = min(1.0, abs(err_med) / (0.5 * self.resize_w))
            norm_ang = min(1.0, abs(ang_med) / 0.7)
            slow_fac = min(1.0, norm_err + norm_ang)
            lin = self.base_linear * (1.0 - self.speed_gain * slow_fac)
            lin = max(self.speed_min, min(self.clamp, lin))

            # 조향 제한 + Slew
            max_steer_now = self.max_steer * (0.6 + 0.4 * (lin / max(1e-3, self.base_linear)))
            if self.steer_slew > 0.0:
                dw_max = self.steer_slew * dt
                w = max(self.prev_w - dw_max, min(self.prev_w + dw_max, w_des))
            else:
                w = w_des
            w = max(-max_steer_now, min(max_steer_now, w))

            tw = Twist()
            tw.linear.x  = lin
            tw.angular.z = w
            self.pub_cmd.publish(tw)

            self.prev_err = err_ctrl
            self.prev_w   = w
            self.prev_t   = now

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("line_tracker_vision")
    node = VisionLineTracker()
    node.run()

