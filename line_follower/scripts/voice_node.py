#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import queue, sys
from google.cloud import speech

# 마이크 대신 파일/스트리밍 구현은 프로젝트 환경에 맞게 교체 가능
import pyaudio

RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class MicrophoneStream:
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16, channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            stream_callback=self._fill_buffer)
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break
            yield b"".join(data)

if __name__ == "__main__":
    rospy.init_node("voice_node")
    pub = rospy.Publisher("/voice/command", String, queue_size=1)

    lang = rospy.get_param("~language_code", "ko-KR")
    client = speech.SpeechClient()

    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=lang,
        max_alternatives=1,
    )
    streaming_config = speech.StreamingRecognitionConfig(
        config=config, interim_results=False, single_utterance=False
    )

    key_go = "가자"
    key_return = "돌아가"

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (speech.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)
        responses = client.streaming_recognize(streaming_config, requests)

        for resp in responses:
            if not resp.results:
                continue
            result = resp.results[0]
            if not result.alternatives:
                continue
            transcript = result.alternatives[0].transcript.strip()
            rospy.loginfo(f"[VOICE] {transcript}")
            if key_go in transcript:
                pub.publish(String(data="가자"))
            elif key_return in transcript:
                pub.publish(String(data="돌아가"))

