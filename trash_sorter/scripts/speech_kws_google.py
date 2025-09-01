#!/usr/bin/env python3
import os, sys, time, queue, signal, threading
import sounddevice as sd
import rospy
from std_msgs.msg import String
from google.cloud import speech
try:
    from audio_common_msgs.msg import AudioData  # 토픽 입력 모드에서 사용
except Exception:
    AudioData = None

# ====== 오디오/장치 ======
LANG_CODE    = "ko-KR"
DEVICE_INDEX = 2          # ← 사용 중인 마이크 인덱스
CHANNELS     = 1

# ====== 퍼블리시 튜닝 ======
COOLDOWN_SEC = 1.0

# ====== 키워드 변형 ======
PLASTIC_VARIANTS = ["플라스틱","플라","플라틱","플락틱","plastic","플라스"]
CAN_VARIANTS     = ["캔","켄","캣","케이","k","K","can","ken"]

# --- 주행 제어 키워드 ---
START_WORDS = ["가자"]
STOP_WORDS  = ["멈춰","스톱","스탑","stop"]
TURN_WORDS  = ["돌아","회전","턴","뒤로 돌아","뒤돌아","돌아서"]
speech_context = speech.SpeechContext(
    phrases=(PLASTIC_VARIANTS + CAN_VARIANTS + TURN_WORDS + START_WORDS + STOP_WORDS),
    boost=15.0
)

def _match_ctrl(text: str) -> str:
    t = text.replace(" ", "").lower()
    if any(w in t for w in START_WORDS): return "start"
    if any(w in t for w in STOP_WORDS):  return "stop"
    if any(w in t for w in TURN_WORDS):  return "turn_around"
    return ""

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
        interim_results=True,
        single_utterance=False
    )

def audio_cb_factory(q: queue.Queue, stop_event: threading.Event):
    def _cb(indata, frames, time_info, status):
        if status:
            rospy.logwarn("[audio] %s", status)
        if stop_event.is_set():
            return
        q.put(bytes(indata))
    return _cb

# --- 요청 제너레이터 2종(버전 호환) ---
def gen_with_config_message(q: queue.Queue, stream_cfg, stop_event: threading.Event):
    yield speech.StreamingRecognizeRequest(streaming_config=stream_cfg)
    while not stop_event.is_set():
        data = q.get()
        if data is None:
            return
        yield speech.StreamingRecognizeRequest(audio_content=data)

def gen_audio_only(q: queue.Queue, stop_event: threading.Event):
    while not stop_event.is_set():
        data = q.get()
        if data is None:
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
        if decided == "stop" or (now - state["last_pub"] >= COOLDOWN_SEC):
            pub.publish(String(decided))
            state["last_pub"] = now
            rospy.loginfo(">> COMMAND: %s | text='%s'", decided, t)

def run_once(client, q, stream_cfg, pub, pub_ctrl, state, stop_event):
    try:
        reqs = gen_audio_only(q, stop_event)  # 최신 방식: config는 인자
        responses = client.streaming_recognize(config=stream_cfg, requests=reqs)
        for resp in responses:
            if stop_event.is_set(): break
            if resp.error.code != 0:
                rospy.logwarn("[stt error] %s", resp.error.message); break
            for result in resp.results:
                txt = result.alternatives[0].transcript if result.alternatives else ""
                if not txt:
                    continue

                # ✅ interim에서도 STOP 나오면 즉시 퍼블리시
                if not result.is_final:
                    t = txt.replace(" ", "").lower()
                    if any(w.replace(" ","") in t for w in STOP_WORDS):
                        pub_ctrl.publish(String("stop"))
                        rospy.loginfo(">> VOICE_CMD (interim STOP): %s", txt.strip())
                        continue  # final 기다리지 않고 바로 STOP

                # 원래 있던 최종 결과 처리
                if result.is_final:
                    decide_and_publish(txt, pub, state)
                    cmd = _match_ctrl(txt)
                    if cmd:
                        pub_ctrl.publish(String(cmd))
                        rospy.loginfo(">> VOICE_CMD: %s | text='%s'", cmd, txt.strip())
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
            txt = result.alternatives[0].transcript if result.alternatives else ""
            if not txt:
                continue

            # ✅ interim에서도 STOP 나오면 즉시 퍼블리시
            if not result.is_final:
                t = txt.replace(" ", "").lower()
                if any(w.replace(" ","") in t for w in STOP_WORDS):
                    pub_ctrl.publish(String("stop"))
                    rospy.loginfo(">> VOICE_CMD (interim STOP): %s", txt.strip())
                    continue  # final 기다리지 않고 바로 STOP

            # 원래 있던 최종 결과 처리
            if result.is_final:
                decide_and_publish(txt, pub, state)
                cmd = _match_ctrl(txt)
                if cmd:
                    pub_ctrl.publish(String(cmd))
                    rospy.loginfo(">> VOICE_CMD: %s | text='%s'", cmd, txt.strip())

def main():
    rospy.init_node("speech_kws_google")
    
    ensure_env()
    client = speech.SpeechClient()

    # ---- 신규 파라미터 (토픽 입력 모드) ----
    use_audio_topic = rospy.get_param("~use_audio_topic", False)
    audio_topic     = rospy.get_param("~audio_topic", "/audio/raw")
    topic_rate      = int(rospy.get_param("~topic_rate", 16000))  # /audio/raw의 샘플레이트(PCM S16LE)

    pub = rospy.Publisher("/bin_command", String, queue_size=1)
    pub_ctrl = rospy.Publisher("/voice_cmd", String, queue_size=1)
    state = {"last_pub": 0.0}


    q  = queue.Queue()
    stop_event = threading.Event()

    # SIGINT/SIGTERM 핸들러
    def handle_signal(signum, frame):
        rospy.loginfo("Signal %s received, shutting down...", signum)
        stop_event.set()
        try:
            q.put_nowait(None)
        except Exception:
            pass
        rospy.signal_shutdown("signal")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    if use_audio_topic:
        # -------- 토픽 입력 모드 --------
        if AudioData is None:
            rospy.logfatal("audio_common_msgs 가 필요합니다. `sudo apt install ros-noetic-audio-common`")
            sys.exit(1)

        rate = topic_rate
        stream_cfg = make_streaming_config(rate)

        def topic_cb(msg):
            if not stop_event.is_set():
                try:
                    # 꽉 차 있으면 오래된 것 버림
                    while q.qsize() > 1:
                        q.get_nowait()
                except Exception:
                    pass
                # audio_capture.launch 설정과 맞춰: S16LE mono @ topic_rate
                q.put(msg.data)

        sub = rospy.Subscriber(audio_topic, AudioData, topic_cb, queue_size=1)
        rospy.loginfo("Google STT (topic mode) @ %dHz, sub=%s → /bin_command", rate, audio_topic)

        try:
            while not rospy.is_shutdown() and not stop_event.is_set():
                try:
                    run_once(client, q, stream_cfg, pub, pub_ctrl, state, stop_event)
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
    else:
        # -------- 기존: 마이크 직접 모드 --------
        rate = get_rate_from_device(DEVICE_INDEX)
        block = max(160, rate // 10)   # ≈0.1초
        stream_cfg = make_streaming_config(rate)
        cb = audio_cb_factory(q, stop_event)

        try:
            with sd.RawInputStream(samplerate=rate, blocksize=block, dtype="int16",
                                   channels=CHANNELS, device=DEVICE_INDEX, callback=cb):
                rospy.loginfo("Google STT (mic mode) ko-KR @ %dHz → /bin_command", rate)
                while not rospy.is_shutdown() and not stop_event.is_set():
                    try:
                        run_once(client, q, stream_cfg, pub, pub_ctrl, state, stop_event)
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
        pass
