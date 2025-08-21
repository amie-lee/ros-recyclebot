#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, queue, re, os, time
import pyaudio
import rospy
from std_msgs.msg import String
from google.cloud import speech

RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class MicStream:
    def __init__(self, rate, chunk, device_index=None):
        self._rate = rate
        self._chunk = chunk
        self._buff = queue.Queue()
        self.closed = True
        self._device_index = device_index

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        kwargs = {
            "format": pyaudio.paInt16,
            "channels": 1,
            "rate": self._rate,
            "input": True,
            "frames_per_buffer": self._chunk,
        }
        if self._device_index is not None:
            kwargs["input_device_index"] = self._device_index
        self._audio_stream = self._audio_interface.open(**kwargs, stream_callback=self._fill_buffer)
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

class VoiceCmdNode:
    def __init__(self):
        rospy.init_node("voice_cmd_google")
        self.pub = rospy.Publisher("/voice_cmd", String, queue_size=3)
        lang = rospy.get_param("~language_code", "ko-KR")
        mic_index = rospy.get_param("~mic_device_index", None)  # 정수 또는 None
        self.mic_index = int(mic_index) if mic_index is not None else None

        self.client = speech.SpeechClient()
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=lang,
            max_alternatives=1,
            enable_automatic_punctuation=False,
        )
        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config,
            interim_results=False,
            single_utterance=False,
        )
        rospy.loginfo("voice_cmd_google ready (lang=%s, mic_index=%s)", lang, str(self.mic_index))

    def run(self):
        with MicStream(RATE, CHUNK, self.mic_index) as stream:
            audio_generator = stream.generator()
            requests = (speech.StreamingRecognizeRequest(audio_content=content) for content in audio_generator)
            responses = self.client.streaming_recognize(self.streaming_config, requests)
            self._listen_print_loop(responses)

    def _listen_print_loop(self, responses):
        for resp in responses:
            if not resp.results:
                continue
            result = resp.results[0]
            if not result.alternatives:
                continue
            transcript = result.alternatives[0].transcript.strip()
            rospy.loginfo("ASR: %s", transcript)

            # 간단 키워드 매칭 (공백/조사 허용)
            t = re.sub(r"\s+", "", transcript)
            if "가자" in t:
                self.pub.publish("go")
            elif "돌아가" in t or "돌아가자" in t:
                self.pub.publish("return")

if __name__ == "__main__":
    try:
        VoiceCmdNode().run()
    except rospy.ROSInterruptException:
        pass

