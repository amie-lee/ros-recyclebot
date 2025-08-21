#!/usr/bin/env python3
import time
import math
import pigpio
import rospy
from std_msgs.msg import String

# ================= 사용자 설정 =================
SERVO_GPIO = 18          # PWM 가능한 BCM 핀: 12, 13, 18, 19 권장
NEUTRAL_DEG = 90
PLASTIC_DEG = 0
CAN_DEG = 180

# 펄스폭(서보에 맞게 보정하세요)
MIN_USEC = 600           # 대략 0°
MAX_USEC = 2400          # 대략 180°

# 모션 프로파일(부드러운 가감속)
MAX_SPEED_DPS = 240.0    # 최대 속도 (deg/s)
MAX_ACCEL_DPS2 = 1200.0  # 최대 가속 (deg/s^2)
CTRL_PERIOD = 0.02       # 제어 주기 (s), 50Hz

# 도달 후 신호 처리
HOLD_MODE = "off_after"  # "hold" | "off" | "off_after"
HOLD_TIME = 0.25         # off_after일 때 유지 시간(s)
# =================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deg_to_usec(deg):
    deg = clamp(deg, 0.0, 180.0)
    return int(MIN_USEC + (MAX_USEC - MIN_USEC) * (deg / 180.0))

class PigpioServo:
    def __init__(self, gpio):
        self.gpio = gpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. 실행: sudo pigpiod")
        self.current_deg = NEUTRAL_DEG
        self._apply_deg(self.current_deg)

    def _apply_deg(self, deg):
        pw = deg_to_usec(deg)
        self.pi.set_servo_pulsewidth(self.gpio, pw)

    def _hold_or_off(self):
        if HOLD_MODE == "off":
            self.pi.set_servo_pulsewidth(self.gpio, 0)
        elif HOLD_MODE == "off_after":
            time.sleep(HOLD_TIME)
            self.pi.set_servo_pulsewidth(self.gpio, 0)

    def move_to(self, target_deg):
        target_deg = clamp(target_deg, 0.0, 180.0)
        start = self.current_deg
        dist = target_deg - start
        if abs(dist) < 0.01:
            # 같은 위치 → 처리만
            self._apply_deg(target_deg)
            self._hold_or_off()
            return

        # 트래페zoid 속도 프로파일 계산
        dir_sign = 1.0 if dist > 0 else -1.0
        dist_abs = abs(dist)

        # 가속/감속 거리
        t_acc = MAX_SPEED_DPS / MAX_ACCEL_DPS2
        d_acc = 0.5 * MAX_ACCEL_DPS2 * t_acc * t_acc

        if 2 * d_acc >= dist_abs:
            # 삼각형 프로파일(최대속도 못 도달)
            t_acc = math.sqrt(dist_abs / MAX_ACCEL_DPS2)
            t_cruise = 0.0
        else:
            # 트래페zoid
            d_cruise = dist_abs - 2 * d_acc
            t_cruise = d_cruise / MAX_SPEED_DPS

        t_total = 2 * t_acc + t_cruise
        t = 0.0
        last_deg = start

        while t < t_total and not rospy.is_shutdown():
            if t < t_acc:
                # 가속 구간
                vel = MAX_ACCEL_DPS2 * t
                moved = 0.5 * MAX_ACCEL_DPS2 * t * t
            elif t < t_acc + t_cruise:
                # 등속 구간
                vel = MAX_SPEED_DPS
                moved = d_acc + MAX_SPEED_DPS * (t - t_acc)
            else:
                # 감속 구간
                td = t - (t_acc + t_cruise)
                vel = max(0.0, MAX_SPEED_DPS - MAX_ACCEL_DPS2 * td)
                moved = d_acc + (t_cruise * MAX_SPEED_DPS) + (MAX_SPEED_DPS * td - 0.5 * MAX_ACCEL_DPS2 * td * td)

            pos = start + dir_sign * moved
            pos = clamp(pos, 0.0, 180.0)

            # 필요할 때만 업데이트(불필요한 전송 줄이기)
            if abs(pos - last_deg) >= 0.2:
                self._apply_deg(pos)
                last_deg = pos

            time.sleep(CTRL_PERIOD)
            t += CTRL_PERIOD

        # 최종 위치 정착
        self._apply_deg(target_deg)
        self.current_deg = target_deg
        self._hold_or_off()

    def stop(self):
        try:
            self.pi.set_servo_pulsewidth(self.gpio, 0)
        finally:
            self.pi.stop()

class ServoControllerNode:
    def __init__(self):
        rospy.init_node("servo_controller_pigpio")
        rospy.on_shutdown(self.on_shutdown)
        self.servo = PigpioServo(SERVO_GPIO)
        rospy.Subscriber("/bin_command", String, self.cb)
        rospy.loginfo("servo_controller_pigpio ready on GPIO %d", SERVO_GPIO)
        # 중립으로 이동
        self.servo.move_to(NEUTRAL_DEG)

    def cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "plastic":
            rospy.loginfo("plastic → 0°")
            self.servo.move_to(PLASTIC_DEG)
        elif cmd == "can":
            rospy.loginfo("can → 180°")
            self.servo.move_to(CAN_DEG)
        else:
            rospy.logwarn("Unknown command: %s", cmd)

    def on_shutdown(self):
        self.servo.stop()

def main():
    try:
        ServoControllerNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr("servo_controller_pigpio failed: %s", e)

if __name__ == "__main__":
    main()

