#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, queue, threading
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import sounddevice as sd
import numpy as np
from google.cloud import speech

class VoiceCmdGate:
    """
    상태머신:
      IDLE --("가자")--> RUN
      RUN --(라인 끊김 timeout)--> IDLE
      언제든 ("멈춰"/"정지") --> IDLE
    """
    def __init__(self):
        rospy.init_node("voice_cmd_gate")

        # ===== 파라미터 =====
        self.keyword_start = rospy.get_param("~keyword_start", "가자")
        stop_kw = rospy.get_param("~keyword_stop", ["멈춰","정지"])
        self.turn_kw = rospy.get_param("~keyword_turn_around", "돌아")      # ★ ADD: 회전 트리거 키워드
        self.turn_dir = rospy.get_param("~turn180_dir", "left")            # ★ ADD: left/right
        self.turn_omega = float(rospy.get_param("~turn180_omega", 0.7))    # ★ ADD: 회전 각속도(rad/s)
        self.turn_duration = float(rospy.get_param("~turn180_duration", 2.8)) # ★ ADD: 회전 시간(초) 캘리브레이션
        self.keyword_stop = stop_kw if isinstance(stop_kw, list) else [stop_kw]

        self.line_lost_timeout = float(rospy.get_param("~line_lost_timeout", 1.0))
        self.turn_abort = False

        # 오디오 파라미터 (sounddevice는 기본 장치가 리샘플/포맷을 꽤 잘 맞춰줌)
        self.rate      = int(rospy.get_param("~mic_rate", 16000))   # STT에 보낼 레이트
        self.channels  = int(rospy.get_param("~channels", 1))
        self.block_ms  = int(rospy.get_param("~block_ms", 100))     # 100ms
        self.device    = rospy.get_param("~mic_device", None)       # None 이면 기본 입력 장치 사용
        # device는 인덱스(int)나 "카드명" 문자열 모두 가능 (sounddevice 규약)

        # 상태
        self.enabled = False
        self.last_line_ts = 0.0
        self.busy = False  

        # ROS 통신
        self.sub_cmd_in = rospy.Subscriber("/line_tracker/cmd_vel", Twist, self.cb_cmd, queue_size=10)
        self.sub_found  = rospy.Subscriber("/line_follower/found", Bool, self.cb_found, queue_size=10)
        self.pub_cmd_out= rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_state  = rospy.Publisher("/voice_cmd/state", String, queue_size=1, latch=True)

        # STT 클라이언트
        self.client = speech.SpeechClient()

        # 스트리밍 설정(테스트 코드와 동일 스타일)
        self.stream_cfg = speech.StreamingRecognitionConfig(
            config=speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=self.rate,   # 우리가 sd에서 뽑아 낸 레이트로 보냄
                language_code="ko-KR",
                enable_automatic_punctuation=False,
                model="default",
                max_alternatives=1,
            ),
            interim_results=True,
            single_utterance=False,
        )

        # 오디오 큐 & 캡처
        self.audio_q: "queue.Queue[bytes]" = queue.Queue()
        self._start_audio_stream()

        # STT 스레드
        self._stt_thread = threading.Thread(target=self._stt_loop, daemon=True)
        self._stt_thread.start()

        rospy.loginfo("voice_cmd_gate (sounddevice) started")
        self._publish_state()

    # ---------- 오디오 캡처 ----------
    def _audio_cb(self, indata, frames, time_info, status):
        if status:
            rospy.logwarn("audio status: %s", status)
        # indata: int16 numpy array shape (frames, channels)
        # LINEAR16 mono로 보낼 것 → channels=1 권장
        self.audio_q.put(bytes(indata))  # 그대로 raw 바이트 전송

    def _start_audio_stream(self):
        blocksize = int(self.rate * self.block_ms / 1000)
        try:
            sd.default.dtype = 'int16'
            self.stream = sd.RawInputStream(
                samplerate=self.rate,
                blocksize=blocksize,
                channels=self.channels,
                dtype='int16',
                device=self.device,     # None이면 기본 장치
                callback=self._audio_cb,
            )
            self.stream.start()
            rospy.loginfo("Mic opened: device=%s rate=%dHz ch=%d block=%d",
                          str(self.device), self.rate, self.channels, blocksize)
        except Exception as e:
            # 장치 기본레이트만 허용하는 경우를 대비, 장치의 기본 samplerate로 재시도
            rospy.logwarn("Mic open failed (%s). Retrying with device default rate.", e)
            dev_info = sd.query_devices(self.device, 'input')
            fallback_rate = int(dev_info['default_samplerate'])
            blocksize = int(fallback_rate * self.block_ms / 1000)
            self.rate = fallback_rate
            # STT쪽 설정도 갱신
            self.stream_cfg.config.sample_rate_hertz = self.rate
            self.stream = sd.RawInputStream(
                samplerate=self.rate,
                blocksize=blocksize,
                channels=self.channels,
                dtype='int16',
                device=self.device,
                callback=self._audio_cb,
            )
            self.stream.start()
            rospy.loginfo("Mic reopened with fallback rate=%dHz", self.rate)

    # ---------- STT ----------
    def _resp_print(self, resp):
        # 디버그용 (원하면 주석 처리)
        for r in resp.results:
            if not r.alternatives: continue
            alt = r.alternatives[0]
            if r.is_final:
                rospy.loginfo("[FINAL] %s", alt.transcript.strip())
            else:
                rospy.logdebug("[...] %s", alt.transcript.strip())

    def _handle_transcript(self, text_no_space: str):
        if not text_no_space: return

        if self.turn_kw.replace(" ", "") in text_no_space:
            if not self.busy:  # 회전 중 중복 방지
                # 회전은 별도 스레드로 실행(오디오/ROS 루프 블로킹 방지)
                threading.Thread(target=self._do_turn_in_place, daemon=True).start()
            return

        if self.keyword_start.replace(" ", "") in text_no_space:
            self._start_run()
            return
        for kw in self.keyword_stop:
            if kw.replace(" ", "") in text_no_space:
                self._stop_and_idle()
                return

    def _requests_gen_audio_only(self):
        while not rospy.is_shutdown():
            data = self.audio_q.get()
            if data is None:
                return
            yield speech.StreamingRecognizeRequest(audio_content=data)

    def _requests_gen_with_config(self):
        # 구버전 호환: 첫 요청에 config 포함
        yield speech.StreamingRecognizeRequest(streaming_config=self.stream_cfg)
        while not rospy.is_shutdown():
            data = self.audio_q.get()
            if data is None:
                return
            yield speech.StreamingRecognizeRequest(audio_content=data)

    def _stt_loop(self):
        while not rospy.is_shutdown():
            try:
                # 최신 시그니처 우선 (테스트 코드와 동일)
                reqs = self._requests_gen_audio_only()
                responses = self.client.streaming_recognize(
                    config=self.stream_cfg,
                    requests=reqs
                )
                for resp in responses:
                    # self._resp_print(resp)  # 필요시 로그
                    for res in resp.results:
                        if not res.alternatives: continue
                        transcript = res.alternatives[0].transcript.strip().replace(" ", "")
                        if transcript:
                            self._handle_transcript(transcript)
            except TypeError as e:
                # 구버전 API fallback
                rospy.logwarn("STT API signature mismatch, fallback. %s", e)
                try:
                    reqs = self._requests_gen_with_config()
                    responses = self.client.streaming_recognize(reqs)  # positional
                    for resp in responses:
                        for res in resp.results:
                            if not res.alternatives: continue
                            transcript = res.alternatives[0].transcript.strip().replace(" ", "")
                            if transcript:
                                self._handle_transcript(transcript)
                except TypeError:
                    # 일부 구버전: keyword only
                    reqs = self._requests_gen_with_config()
                    responses = self.client.streaming_recognize(requests=reqs)
                    for resp in responses:
                        for res in resp.results:
                            if not res.alternatives: continue
                            transcript = res.alternatives[0].transcript.strip().replace(" ", "")
                            if transcript:
                                self._handle_transcript(transcript)
            except Exception as e:
                rospy.logwarn("STT stream error: %s (retry in 1s)", e)
                time.sleep(1.0)

    # ---------- ROS 콜백 & 상태 ----------
    def cb_cmd(self, msg: Twist):
        if self.enabled and not self.busy:
            self.pub_cmd_out.publish(msg)

    def cb_found(self, msg: Bool):
        if msg.data:
            self.last_line_ts = time.time()
        else:
            if self.enabled and (time.time() - self.last_line_ts) > self.line_lost_timeout:
                rospy.loginfo("Line lost → STOP")
                self._stop_and_idle()

    def _publish_state(self):
        if self.busy:
            s = "TURNING"
        else:
            s = "RUN" if self.enabled else "IDLE"
        self.pub_state.publish(String(data=s))

    def _start_run(self):
        if not self.enabled:
            self.enabled = True
            self.last_line_ts = time.time()
            self._publish_state()
            rospy.loginfo("[voice] RUN")

    def _stop_and_idle(self):
        if self.enabled or self.busy:
            self.enabled = False
            self.turn_abort = True
        self.pub_cmd_out.publish(Twist())  # 즉시 정지
        self._publish_state()
        rospy.loginfo("[voice] IDLE")

    def _do_turn_in_place(self):
        """제자리 180° 회전(블로킹). 끝나면 이전 상태로 복귀."""
        # 상태 저장
        prev_enabled = self.enabled
        self.busy = True          # 입력 게이트 일시 차단
        self.enabled = False      # 라인 추종 릴레이 잠시 OFF
        self._publish_state()
        rospy.loginfo("[voice] TURNING 180 start")

        # 방향 부호 결정
        if str(self.turn_dir).lower() == "left":
            s = +1.0
        elif str(self.turn_dir).lower() == "right":
            s = -1.0
        else:
            s = +1.0  # 기본 left

        # 회전 수행
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = s * float(self.turn_omega)

        rate = rospy.Rate(50)  # 50Hz
        t_start = rospy.get_time()
        while (rospy.get_time() - t_start) < self.turn_duration and not rospy.is_shutdown():
            if self.turn_abort:
                rospy.loginfo("[voice] TURNING aborted by STOP")
                break
            self.pub_cmd_out.publish(t)
            rate.sleep()
        self.turn_abort = False    

        # 정지
        self.pub_cmd_out.publish(Twist())
        rospy.sleep(0.1)

        # 상태 복구: 이전에 RUN 상태였으면 다시 RUN으로
        self.enabled = prev_enabled
        self.busy = False
        self._publish_state()
        rospy.loginfo("[voice] TURNING 180 done → %s", "RUN" if self.enabled else "IDLE")


def main():
    # 인증 파일 체크(환경변수)
    cred = os.environ.get("GOOGLE_APPLICATION_CREDENTIALS")
    if not cred or not os.path.isfile(cred):
        rospy.logwarn("GOOGLE_APPLICATION_CREDENTIALS not set or missing.")

    VoiceCmdGate()
    rospy.spin()

if __name__ == "__main__":
    main()
