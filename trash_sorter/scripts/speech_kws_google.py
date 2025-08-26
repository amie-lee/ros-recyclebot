#!/usr/bin/env python3
import os, sys, time, queue, signal, threading
import sounddevice as sd
import rospy
from std_msgs.msg import String
from google.cloud import speech

# ====== 오디오/장치 ======
LANG_CODE    = "ko-KR"
DEVICE_INDEX = 2          # ← 사용 중인 마이크 인덱스
CHANNELS     = 1

# ====== 퍼블리시 튜닝 ======
COOLDOWN_SEC = 1.0

# ====== 키워드 변형 ======
PLASTIC_VARIANTS = ["플라스틱","플라","플라틱","플락틱","plastic","플라스"]
CAN_VARIANTS     = ["캔","켄","캣","케이","k","K","can","ken"]

def norm(s: str) -> str: return s.replace(" ", "").lower()
PLASTIC_N = [norm(x) for x in PLASTIC_VARIANTS]
CAN_N     = [norm(x) for x in CAN_VARIANTS]

def contains_any(text: str, vocab_norm) -> bool:
    t = norm(text)
    return any(v in t for v in vocab_norm)

def ensure_env():
    cred = os.environ.get("GOOGLE_APPLICATION_CREDENTIALS")
    if not cred or not os.path.isfile(cred):
        rospy.logfatal("GOOGLE_APPLICATION_CREDENTIALS 미설정/경로 오류")
        sys.exit(1)

def get_rate_from_device(dev_idx: int) -> int:
    info = sd.query_devices(dev_idx)
    rate = int(round(info["default_samplerate"]))  # ex) 48000 또는 44100
    rospy.loginfo("Using input device #%d '%s' @ %dHz", dev_idx, info["name"], rate)
    return rate

def make_streaming_config(rate_hz: int):
    speech_context = speech.SpeechContext(
        phrases=PLASTIC_VARIANTS + CAN_VARIANTS, boost=15.0
    )
    recog_cfg = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=rate_hz,
        language_code=LANG_CODE,
        enable_automatic_punctuation=False,
        model="default",
        speech_contexts=[speech_context],
    )
    return speech.StreamingRecognitionConfig(
        config=recog_cfg,
        interim_results=False,
        single_utterance=False
    )

def audio_cb_factory(q: queue.Queue, stop_event: threading.Event):
    def _cb(indata, frames, time_info, status):
        if status:
            rospy.logwarn("[audio] %s", status)
        # stop이면 더 이상 밀어넣지 않음
        if stop_event.is_set():
            return
        q.put(bytes(indata))
    return _cb

# --- 요청 제너레이터 2종(버전 호환) ---
def gen_with_config_message(q: queue.Queue, stream_cfg, stop_event: threading.Event):
    yield speech.StreamingRecognizeRequest(streaming_config=stream_cfg)
    while not stop_event.is_set():
        data = q.get()
        if data is None:  # 종료 신호
            return
        yield speech.StreamingRecognizeRequest(audio_content=data)

def gen_audio_only(q: queue.Queue, stop_event: threading.Event):
    while not stop_event.is_set():
        data = q.get()
        if data is None:  # 종료 신호
            return
        yield speech.StreamingRecognizeRequest(audio_content=data)

def decide_and_publish(transcript: str, pub, state):
    if not transcript: return
    t = transcript.strip()
    hit_can = contains_any(t, CAN_N)
    hit_pla = contains_any(t, PLASTIC_N)
    decided = "can" if (hit_can and not hit_pla) else ("plastic" if (hit_pla and not hit_can) else None)
    if decided:
        now = time.time()
        if now - state["last_pub"] >= COOLDOWN_SEC:
            pub.publish(String(decided))
            state["last_pub"] = now
            rospy.loginfo(">> COMMAND: %s | text='%s'", decided, t)

def run_once(client, q, stream_cfg, pub, state, stop_event):
    try:
        reqs = gen_audio_only(q, stop_event)  # 최신 방식: config는 인자
        responses = client.streaming_recognize(config=stream_cfg, requests=reqs)
        for resp in responses:
            if stop_event.is_set(): break
            if resp.error.code != 0:
                rospy.logwarn("[stt error] %s", resp.error.message); break
            for result in resp.results:
                if result.is_final and result.alternatives:
                    decide_and_publish(result.alternatives[0].transcript, pub, state)
        return
    except TypeError:
        rospy.logdebug("fallback to legacy API")

    # 구버전 방식
    reqs = gen_with_config_message(q, stream_cfg, stop_event)
    try:
        responses = client.streaming_recognize(reqs)
    except TypeError:
        responses = client.streaming_recognize(requests=reqs)
    for resp in responses:
        if stop_event.is_set(): break
        if resp.error.code != 0:
            rospy.logwarn("[stt error] %s", resp.error.message); break
        for result in resp.results:
            if result.is_final and result.alternatives:
                decide_and_publish(result.alternatives[0].transcript, pub, state)

def main():
    rospy.init_node("speech_kws_google")
    ensure_env()
    client = speech.SpeechClient()

    rate = get_rate_from_device(DEVICE_INDEX)
    block = max(160, rate // 10)   # ≈0.1초
    stream_cfg = make_streaming_config(rate)

    pub = rospy.Publisher("/bin_command", String, queue_size=1)
    state = {"last_pub": 0.0}

    q  = queue.Queue()
    stop_event = threading.Event()

    # SIGINT/SIGTERM 핸들러
    def handle_signal(signum, frame):
        rospy.loginfo("Signal %s received, shutting down...", signum)
        stop_event.set()
        try:
            q.put_nowait(None)  # 제너레이터 종료 유도
        except Exception:
            pass
        rospy.signal_shutdown("signal")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    cb = audio_cb_factory(q, stop_event)

    try:
        with sd.RawInputStream(samplerate=rate, blocksize=block, dtype="int16",
                               channels=CHANNELS, device=DEVICE_INDEX, callback=cb):
            rospy.loginfo("Google STT listening (ko-KR, %dHz) → /bin_command", rate)
            while not rospy.is_shutdown() and not stop_event.is_set():
                try:
                    run_once(client, q, stream_cfg, pub, state, stop_event)
                except Exception as e:
                    if stop_event.is_set(): break
                    rospy.logwarn("[warn] stream restart: %s", e)
                    time.sleep(0.3)
    finally:
        stop_event.set()
        try:
            q.put_nowait(None)
        except Exception:
            pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # 혹시 모를 상위 레벨 키보드 인터럽트
        pass
