#!/usr/bin/env python3
import os, json, queue
import rospy, sounddevice as sd
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

MODEL_PATH = os.environ.get("VOSK_MODEL", os.path.expanduser("~/models/vosk-model-small-ko"))
SAMPLE_RATE = 16000
CHANNELS = 1

PLASTIC = ["플라스틱","플라","plastic"]
CAN     = ["캔","켄","캣","케이","k","can","ken"]

def norm(s): return s.replace(" ","").lower()

def main():
    rospy.init_node("speech_kws_min")
    pub = rospy.Publisher("/bin_command", String, queue_size=1)
    model = Model(MODEL_PATH)
    rec = KaldiRecognizer(model, SAMPLE_RATE)  # ◀️ Grammar 없음
    q = queue.Queue()

    def cb(indata, frames, t, status):
        if status: rospy.logwarn("audio %s", status)
        q.put(bytes(indata))

    with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=8000, dtype='int16',
                           channels=CHANNELS, callback=cb):
        rospy.loginfo("speech_kws_min ready")
        while not rospy.is_shutdown():
            data = q.get()
            if not rec.AcceptWaveform(data):
                continue
            res = json.loads(rec.Result() or "{}")
            text = (res.get("text") or "").strip()
            if not text: 
                continue
            t = norm(text)
            if any(norm(w) in t for w in CAN):
                pub.publish(String("can"))
                rospy.loginfo("CMD: can | text=%s", text)
            elif any(norm(w) in t for w in PLASTIC):
                pub.publish(String("plastic"))
                rospy.loginfo("CMD: plastic | text=%s", text)

if __name__ == "__main__":
    main()

