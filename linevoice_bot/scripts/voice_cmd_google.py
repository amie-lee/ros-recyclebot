#!/usr/bin/env python3
import os
import time
import json
import rospy
import speech_recognition as sr
from std_msgs.msg import String

class VoiceNode:
    def __init__(self):
        vcfg = rospy.get_param("~voice")
        self.cooldown = float(vcfg.get("cooldown_sec", 2.0))
        self.phrase_time_limit = int(vcfg.get("phrase_time_limit_sec", 4))
        cred_path = vcfg.get("credentials_json_path", "")

        # 구글 키 로드
        if cred_path and os.path.exists(cred_path):
            with open(cred_path, "r") as f:
                self.credentials_json = f.read()
        else:
            # 환경변수도 허용
            envp = os.environ.get("GOOGLE_APPLICATION_CREDENTIALS", "")
            if envp and os.path.exists(envp):
                with open(envp, "r") as f:
                    self.credentials_json = f.read()
            else:
                rospy.logerr("Google credentials not found. Set params or GOOGLE_APPLICATION_CREDENTIALS.")
                self.credentials_json = None

        self.pub = rospy.Publisher("voice_cmd", String, queue_size=3)
        self.last_ts = 0.0

        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = True
        self.mic = sr.Microphone()

        rospy.loginfo("Voice node ready. Say '가자' or '돌아가'.")

    def run(self):
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source, duration=1.0)

        while not rospy.is_shutdown():
            try:
                with self.mic as source:
                    audio = self.r.listen(source, timeout=None, phrase_time_limit=self.phrase_time_limit)
                if self.credentials_json:
                    text = self.r.recognize_google_cloud(
                        audio_data=audio,
                        credentials_json=self.credentials_json,
                        language="ko-KR"
                    )
                else:
                    # 마지막 수단: 무료 Google Web API (비권장/불안정)
                    text = self.r.recognize_google(audio_data=audio, language="ko-KR")

                text = text.strip()
                if text:
                    rospy.loginfo("Heard: %s", text)
                    now = time.time()
                    if now - self.last_ts < self.cooldown:
                        continue

                    if "가자" in text:
                        self.pub.publish(String("GO"))
                        self.last_ts = now
                    elif "돌아가" in text:
                        self.pub.publish(String("RETURN"))
                        self.last_ts = now

            except sr.UnknownValueError:
                pass
            except sr.RequestError as e:
                rospy.logerr("Speech API error: %s", str(e))
                rospy.sleep(1.0)
            except Exception as e:
                rospy.logerr("Voice exception: %s", str(e))
                rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("voice_cmd_google")
    VoiceNode().run()

