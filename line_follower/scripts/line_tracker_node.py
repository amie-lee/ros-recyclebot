#!/usr/bin/env python3
import rospy, cv2, numpy as np
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_err = rospy.Publisher("/line_tracker/error", Float32, queue_size=10)
        self.pub_stat = rospy.Publisher("/line_tracker/status", String, queue_size=10)
        self.sub_img = rospy.Subscriber("/camera/image_raw", Image, self.cb_img, queue_size=1)
        self.lost_count = 0
        self.finish_timeout = rospy.get_param("~finish_timeout", 30.0)  # s
        self.last_seen = rospy.Time.now()

    def cb_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]
        roi = img[int(h*0.6):, :]  # 하단 40%만
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, bw = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        M = cv2.moments(bw)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            err = (cx - (w/2.0)) / (w/2.0)  # -1~1
            self.pub_err.publish(Float32(data=float(err)))
            self.pub_stat.publish(String(data="tracking"))
            self.last_seen = rospy.Time.now()
        else:
            self.pub_stat.publish(String(data="lost"))
            # 라인 미검출이 오래 지속되면 finish로 간주(엔드마커 가정)
            if (rospy.Time.now() - self.last_seen).to_sec() > self.finish_timeout:
                self.pub_stat.publish(String(data="finish"))

if __name__ == "__main__":
    rospy.init_node("line_tracker")
    LineTracker()
    rospy.spin()

