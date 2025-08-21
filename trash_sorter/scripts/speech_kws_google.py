#!/usr/bin/env python3
# Google Cloud STT(ko-KR) → "plastic"/"can" 판정 → /bin_command 퍼블리시
# - streaming_recognize의 시그니처 변화(config/requests) 양쪽 모두 자동 대응
# - 최종 인식만 처리, 쿨다운으로 중복 억제, 부분 포함 매칭

import os, sys, time, queue
import sounddevice as sd
import rospy
from std_msgs.msg import String
from google.cloud import speech

# ====== 오디오/언어 ======
LANG_CODE   = "ko-KR"
RATE        = 16000
CHANNELS    = 1
BLOCK       = 1600             # 0.1s 단위 전송

# ====== 퍼블리시 튜닝 ======
COOLDOWN_SEC = 1.0

# ====== 키워드 변형 ======
PLASTIC_VARIANTS = ["플라스틱","플라","플라틱","플락틱","plastic","플라스"]
CAN_VARIANTS     = ["캔","켄","캣","케이","k","K","can","ken"]

def norm(s: str) -> str:
    return s.replace(" ", "").lower()

PLASTIC_N = [norm(x) for x in PLASTIC_VARIANTS]
CAN_N     = [norm(x) for x in CAN_VARIANTS]

def contains_any(text: str, vocab_norm) -> bool:
    t = norm(text)
    return any(v in t for v in vocab_norm)

def ensure_env():
    cred = os.environ.get("GOOGLE_APPLICATION_CREDENTIALS")
    if not cred or not os.path.isfile(cred):
        rospy.logfatal("GOOGLE_APPLICATION_CREDENTIALS 미설정 또는 파일 없음")
        rospy.logfatal("예) export GOOGLE_APPLICATION_CREDENTIALS=/home/ssm/gcloud-sa.json")
        sys.exit(1)

def make_streaming_config():
    # phrase hints로 키워드 가중치 ↑ (너무 높이면 오탐↑, 10~20 사이 권장)
    speech_context = speech.SpeechContext(
        phrases=PLASTIC_VARIANTS + CAN_VARIANTS,
        boost=15.0
    )
    recog_cfg = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=LANG_CODE,
        enable_automatic_punctuation=False,
        model="default",
        speech_contexts=[speech_context],
    )
    return speech.StreamingRecognitionConfig(
        config=recog_cfg,
        interim_results=False,   # 최종 인식만
        single_utterance=False
    )

def audio_cb_factory(q: queue.Queue):
    def _cb(indata, frames, time_info, status):
        if status:
            rospy.logwarn(f"[audio] {status}")
        q.put(bytes(indata))
    return _cb

# --- 요청 제너레이터 2종 (버전별 호환) ---
def gen_with_config_message(q: queue.Queue, stream_cfg):
    """구버전 방식: 첫 메시지에 streaming_config 포함"""
    yield speech.StreamingRecognizeRequest(streaming_config=stream_cfg)
    while True:
        data = q.get()
        if data is None:
            return
        yield speech.StreamingRecognizeRequest(audio_content=data)

def gen_audio_only(q: queue.Queue):
    """신버전 방식: 함수 인자로 config 전달 → 여기선 audio만 전송"""
    while True:
        data = q.get()
        if data is None:
            return
        yield speech.StreamingRecognizeRequest(audio_content=data)

def decide_and_publish(transcript: str, pub, state):
    """문자열에 기반해 plastic/can 판정 → /bin_command 퍼블리시(쿨다운)"""
    if not transcript:
        return
    t = transcript.strip()
    # 양쪽 다 포함되면 모호 → 무시
    hit_can = contains_any(t, CAN_N)
    hit_pla = contains_any(t, PLASTIC_N)

    decided = None
    if hit_can and not hit_pla:
        decided = "can"
    elif hit_pla and not hit_can:
        decided = "plastic"

    if decided:
        now = time.time()
        if now - state["last_pub"] >= COOLDOWN_SEC:
            pub.publish(String(decided))
            state["last_pub"] = now
            rospy.loginfo(">> COMMAND: %s | text='%s'", decided, t)
        else:
            rospy.logdebug("Cooldown active; skip publish (%0.2fs left)",
                           COOLDOWN_SEC - (now - state["last_pub"]))
    else:
        rospy.logdebug("No decision | text='%s'", t)

def run_once(client, q, stream_cfg, pub, state):
    """라이브러리 버전에 따라 두 방식 시도:
       1) 최신: streaming_recognize(config=..., requests=audio_only)
       2) 구식: streaming_recognize(requests=config_first_then_audio)
    """
    # 1) 최신 방식
    try:
        reqs = gen_audio_only(q)
        responses = client.streaming_recognize(config=stream_cfg, requests=reqs)
        for resp in responses:
            if resp.error.code != 0:
                rospy.logwarn("[stt error] %s", resp.error.message)
                break
            for result in resp.results:
                if result.is_final and result.alternatives:
                    decide_and_publish(result.alternatives[0].transcript, pub, state)
        return
    except TypeError as e:
        # 함수 시그니처 불일치 → 구버전으로 재시도
        rospy.logdebug("fallback to legacy API due to TypeError: %s", e)

    # 2) 구버전 방식
    reqs = gen_with_config_message(q, stream_cfg)
    try:
        responses = client.streaming_recognize(reqs)            # positional
    except TypeError:
        responses = client.streaming_recognize(requests=reqs)   # keyword

    for resp in responses:
        if resp.error.code != 0:
            rospy.logwarn("[stt error] %s", resp.error.message)
            break
        for result in resp.results:
            if result.is_final and result.alternatives:
                decide_and_publish(result.alternatives[0].transcript, pub, state)

def main():
    rospy.init_node("speech_kws_google")
    ensure_env()
    client = speech.SpeechClient()
    stream_cfg = make_streaming_config()

    pub = rospy.Publisher("/bin_command", String, queue_size=1)
    state = {"last_pub": 0.0}

    q  = queue.Queue()
    cb = audio_cb_factory(q)

    # 필요 시 특정 장치 지정: device=<번호>
    with sd.RawInputStream(samplerate=RATE, blocksize=BLOCK, dtype="int16",
                           channels=CHANNELS, callback=cb):
        rospy.loginfo("Google STT listening (ko-KR, %dHz) → /bin_command", RATE)
        while not rospy.is_shutdown():
            try:
                run_once(client, q, stream_cfg, pub, state)
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logwarn("[warn] stream restart: %s", e)
                time.sleep(0.3)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

