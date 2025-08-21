#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# ===== 사용자 설정 =====
SERVO_GPIO = 18        # PWM 권장: 12,13,18,19 (BCM)
FREQ_HZ = 50           # 표준 50Hz
NEUTRAL_DEG = 90
PLASTIC_DEG = 0
CAN_DEG = 180

# 서보별 보정(끝단 떨림 줄이려면 범위를 약간 좁혀보세요)
MIN_USEC = 600         # 대략 0°
MAX_USEC = 2400        # 대략 180°
# 이동 후 신호 처리 방법
SIGNAL_MODE = "off_after"     # "hold", "off", "off_after"
SIGNAL_HOLD_SEC = 0.25        # off_after일 때 유지 시간

PERIOD_USEC = 1000000.0 / FREQ_HZ

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deg_to_usec(deg):
    deg = clamp(deg, 0, 180)
    return MIN_USEC + (MAX_USEC - MIN_USEC) * (deg / 180.0)

def usec_to_duty_percent(usec):
    return (usec / PERIOD_USEC) * 100.0

class ServoGPIO:
    def __init__(self, pin, freq_hz):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, freq_hz)
        self.pwm.start(0.0)
        time.sleep(0.1)
        self.move_to(NEUTRAL_DEG, init=True)

    def move_to(self, deg, init=False):
        pw = deg_to_usec(deg)
        duty = usec_to_duty_percent(pw)
        self.pwm.ChangeDutyCycle(duty)
        rospy.loginfo("Servo -> %3d deg (pulse=%4dus, duty=%.2f%%)", deg, int(pw), duty)

        # 이동 시간 (보수적으로)
        # 일반 소형 서보 60°/0.12~0.18s 가이드 → 각도차에 비례해 가산
        base = 0.20
        per_deg = 0.003
        wait = base + per_deg * abs(deg - NEUTRAL_DEG if init else 0)
        time.sleep(wait)

        if SIGNAL_MODE == "off":
            self.pwm.ChangeDutyCycle(0.0)
        elif SIGNAL_MODE == "off_after":
            time.sleep(SIGNAL_HOLD_SEC)
            self.pwm.ChangeDutyCycle(0.0)
        # "hold"면 유지

    def stop(self):
        try:
            self.pwm.ChangeDutyCycle(0.0)
            time.sleep(0.05)
            self.pwm.stop()
        finally:
            GPIO.cleanup(self.pin)

class ServoControllerNode:
    def __init__(self):
        rospy.init_node("servo_controller_gpio")
        self.servo = ServoGPIO(SERVO_GPIO, FREQ_HZ)
        rospy.Subscriber("/bin_command", String, self.cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("servo_controller_gpio ready on GPIO %d (50Hz)", SERVO_GPIO)

    def cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "plastic":
            self.servo.move_to(PLASTIC_DEG)
        elif cmd == "can":
            self.servo.move_to(CAN_DEG)
        else:
            rospy.logwarn("Unknown command: %s", cmd)

    def on_shutdown(self):
        self.servo.stop()

def main():
    ServoControllerNode()
    rospy.spin()

if __name__ == "__main__":
    main()

