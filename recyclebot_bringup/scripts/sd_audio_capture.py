#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, queue, threading, time
import sounddevice as sd
from audio_common_msgs.msg import AudioData

"""
sounddevice 기반 오디오 캡처 → /audio/raw 퍼블리시
- GStreamer(audio_capture) 대신 사용
- PCM S16LE, mono
ROS params:
~device_index (int, default: -1)  # -1이면 시스템 기본 입력
~sample_rate  (int, default: 16000)
~channels     (int, default: 1)
~block_ms     (int, default: 100) # 블록 길이(ms) - 레이턴시/부하 트레이드오프
~topic        (str, default: /audio/raw)
"""

def main():
    rospy.init_node("sd_audio_capture")

    dev_idx = int(rospy.get_param("~device_index", -1))
    rate    = int(rospy.get_param("~sample_rate", 16000))
    ch      = int(rospy.get_param("~channels", 1))
    blockms = int(rospy.get_param("~block_ms", 100))
    topic   = rospy.get_param("~topic", "/audio/raw")

    pub = rospy.Publisher(topic, AudioData, queue_size=50)

    q = queue.Queue()
    stop = threading.Event()
    blocksize = max(int(rate * blockms / 1000), 160)  # 최소 160 샘플

    # 디바이스 정보 로그
    try:
        if dev_idx >= 0:
            info = sd.query_devices(dev_idx)
        else:
            info = sd.query_devices(kind='input')
        rospy.loginfo("Input device: %s", info)
    except Exception as e:
        rospy.logfatal("오디오 장치 조회 실패: %s", e)
        raise SystemExit(1)

    def cb(indata, frames, time_info, status):
        if status:
            rospy.logwarn("[audio] %s", status)
        if stop.is_set():
            return
        # 오래된 오디오 버리고 최신 블록만 유지
        try:
            while q.qsize() > 1:   # 큐에 이미 쌓인 게 있으면 버림
                q.get_nowait()
        except Exception:
            pass

        q.put(bytes(indata))       # 최신 블록 넣기
        

    rospy.loginfo("sd_audio_capture start: dev=%s rate=%d ch=%d block=%d",
                  (dev_idx if dev_idx>=0 else "default"), rate, ch, blocksize)

    try:
        with sd.RawInputStream(samplerate=rate, blocksize=blocksize,
                               dtype='int16', channels=ch,
                               device=(dev_idx if dev_idx>=0 else None),
                               callback=cb):
            while not rospy.is_shutdown() and not stop.is_set():
                try:
                    data = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                msg = AudioData()
                msg.data = data  # raw PCM bytes
                pub.publish(msg)
    except Exception as e:
        rospy.logfatal("오디오 캡처 실패: %s", e)
        raise SystemExit(1)
    finally:
        stop.set()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
