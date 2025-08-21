#!/usr/bin/env python3
import json, queue, time, os
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import rospy
from std_msgs.msg import String

# ===== 경로/오디오 =====
MODEL_PATH  = os.environ.get("VOSK_MODEL", os.path.expanduser("~/models/vosk-model-small-ko"))
SAMPLE_RATE = 16000
CHANNELS    = 1

# ===== 키워드 변형 =====
PLASTIC_VARIANTS = ["플라스틱","플라","플라틱","플락틱","plastic","플라스"]
CAN_VARIANTS     = ["캔","켄","캣","케이","k","can","ken"]
WAKE_VARIANTS    = ["시작"]  # 필요시 추가

GRAMMAR_LIST = sorted(set(PLASTIC_VARIANTS + CAN_VARIANTS + WAKE_VARIANTS))

def norm(s:str)->str:
    return s.replace(" ","").lower()
PLASTIC_N = [norm(x) for x in PLASTIC_VARIANTS]
CAN_N     = [norm(x) for x in CAN_VARIANTS]
WAKE_N    = [norm(x) for x in WAKE_VARIANTS]

# ===== 튜닝 파라미터 =====
COOLDOWN_SEC       = 1.2   # 연속 발행 최소 간격
MIN_CONF_SUM       = 0.50  # 후보(플라/캔) 합산 신뢰도 최소
MARGIN_DIFF        = 0.10  # 두 후보 점수 차 최소
ACTIVE_WINDOW_SEC  = 3.0   # 웨이크 후 유효 시간
ALLOW_MULTIPLE_CMD = False # True면 창 내 여러 번 명령 허용

def score_from_words(words):
    p_sum = c_sum = 0.0
    for w in words or []:
        t = norm(w.get("word",""))
        conf = float(w.get("conf",0.0))
        if any(v in t for v in CAN_N):     c_sum += conf
        if any(v in t for v in PLASTIC_N): p_sum += conf
    return p_sum, c_sum

def decide_from_text_and_words(text, words):
    p_sum, c_sum = score_from_words(words)
    if max(p_sum, c_sum) >= MIN_CONF_SUM and abs(p_sum - c_sum) >= MARGIN_DIFF:
        return "plastic" if p_sum > c_sum else "can"
    t = norm(text)
    hit_can = any(v in t for v in CAN_N)
    hit_pla = any(v in t for v in PLASTIC_N)
    if hit_can and not hit_pla: return "can"
    if hit_pla and not hit_can: return "plastic"
    return None

def contains_wake(text, words):
    t = norm(text)
    if any(v in t for v in WAKE_N): return True
    for w in words or []:
        if any(v in norm(w.get("word","")) for v in WAKE_N):
            return True
    return False

def main():
    rospy.init_node("speech_kws")
    pub = rospy.Publisher("/bin_command", String, queue_size=1)

    if not os.path.isdir(MODEL_PATH):
        rospy.logerr("Vosk model not found at %s", MODEL_PATH); raise SystemExit(1)

    model = Model(MODEL_PATH)
    rec   = KaldiRecognizer(model, SAMPLE_RATE, json.dumps(GRAMMAR_LIST))

    q = queue.Queue()
    last_pub_t  = 0.0
    armed_until = 0.0

    def audio_cb(indata, frames, time_info, status):
        if status: rospy.logwarn("Audio status: %s", status)
        q.put(bytes(indata))

    with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=8000, dtype='int16',
                           channels=CHANNELS, callback=audio_cb):
        rospy.loginfo("speech_kws with wake gating ready")
        while not rospy.is_shutdown():
            data = q.get()
            if not rec.AcceptWaveform(data):
                _ = rec.PartialResult()  # partial 무시
                continue

            res   = json.loads(rec.Result() or "{}")
            text  = (res.get("text") or "").strip()
            words = res.get("result", [])

            if not text:
                continue

            now = time.time()

            # 1) 웨이크워드 감지 → 창 오픈
            if contains_wake(text, words):
                armed_until = now + ACTIVE_WINDOW_SEC
                rospy.loginfo("** WAKE OK ** window=%.1fs (text='%s')", ACTIVE_WINDOW_SEC, text)
                continue

            # 2) 창이 열려 있는 동안만 명령 판정
            if now <= armed_until:
                decided = decide_from_text_and_words(text, words)
                if decided:
                    if now - last_pub_t >= COOLDOWN_SEC:
                        pub.publish(String(decided))
                        last_pub_t = now
                        p_sum, c_sum = score_from_words(words)
                        rospy.loginfo(">> COMMAND: %s | conf_sum P=%.2f C=%.2f | text='%s'",
                                      decided, p_sum, c_sum, text)
                        if not ALLOW_MULTIPLE_CMD:
                            armed_until = 0.0  # 한 번 쏘고 닫기
                    else:
                        rospy.logdebug("Cooldown active; skip cmd")
                else:
                    rospy.loginfo("In window, no decision (text='%s')", text)
            else:
                # 창 닫힌 상태에선 무시
                rospy.logdebug("Idle(no wake): '%s'", text)

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: pass

