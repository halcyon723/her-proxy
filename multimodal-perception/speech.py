"""语音识别模块：麦克风录音 + 调用豆包语音（火山引擎）转文字。

与姿态识别并行：姿态管"身体在做什么"，语音管"人在说什么"。
输出：识别出的文字 + 原始响应（含 token 用量，方便排查）。

设计要点：
1. 录音用 sounddevice（跨平台、比 pyaudio 好装），采样率 16kHz 足够语音识别用。
2. 音频编码成 WAV 内存流，POST 到豆包语音「录音文件识别 2.0」接口（提交+查询），
   不落盘敏感录音文件。
3. 所有参数走 config.py，以后要换模型/换 key 只改配置不动代码。
4. SpeechListener 做实时连续识别：用 VAD（语音活动检测）自动判断一句话的起止，
   说完静音一小会儿就自动触发识别，不用手动按按钮，也切掉了前后静音让识别更准更快。
"""

import base64
import io
import logging
import math
import re
import threading
import time
import uuid
import wave
from collections import deque

import numpy as np
import requests

import config

logger = logging.getLogger("speech")
if not logger.handlers:
    _h = logging.StreamHandler()
    _h.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
    logger.addHandler(_h)
    logger.setLevel(logging.INFO)

# 复用 HTTP 连接，避免每次提交/查询都重建 TLS 握手，降低识别延迟
_ASR_SESSION = requests.Session()

# 浮点计时容差：0.1 秒逐块累加会有 ±1e-15 误差，0.8 这类阈值可能永远差一点点够不到，
# 用 1e-6 秒的容差消除比较抖动（1 微秒对断句毫无影响）。
_FLT_EPS = 1e-6


def record_audio(samplerate=16000, channels=1):
    """从麦克风录音：按回车开始，再按回车结束。

    返回 (int16 numpy 数组, 采样率)。数组形状 (样本数, 通道数)。
    """
    import sounddevice as sd

    input("🔴 按回车开始录音，然后说话...")
    chunks = []

    def callback(indata, frames, time_info, status):
        chunks.append(indata.copy())

    stream = sd.InputStream(samplerate=samplerate, channels=channels,
                            dtype="int16", callback=callback)
    with stream:
        input("⏺  录音中...（说完后按回车结束）")
    if not chunks:
        return np.zeros((0, channels), dtype=np.int16), samplerate
    audio = np.concatenate(chunks, axis=0)
    return audio, samplerate


def _to_wav_bytes(audio, samplerate):
    """把 int16 numpy 数组编码成 WAV 字节流，供 requests 直接上传。"""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1 if audio.ndim == 1 else audio.shape[1])
        w.setsampwidth(2)
        w.setframerate(samplerate)
        w.writeframes(audio.astype(np.int16).tobytes())
    buf.seek(0)
    return buf


def _denoise(audio, samplerate, cutoff=80.0):
    """送识别前简单降噪：一阶高通滤掉低频（空调/风扇/电流嗡嗡声），保留语音主频带。"""
    if audio is None or len(audio) == 0:
        return audio
    x = audio.astype(np.float32)
    dt = 1.0 / samplerate
    rc = 1.0 / (2 * np.pi * cutoff)
    alpha = rc / (rc + dt)
    y = np.empty_like(x)
    prev_x = 0.0
    prev_y = 0.0
    for n in range(len(x)):
        y[n] = alpha * (prev_y + x[n] - prev_x)
        prev_x = x[n]
        prev_y = y[n]
    return y.astype(np.int16)


def transcribe(audio, samplerate, model=None, language=None):
    """把音频送去豆包语音「录音文件识别 2.0」识别（提交 → 轮询查询）。

    返回 dict：
        {ok, text, model, usage, error}
    ok=True 时 text 是识别出的文字；ok=False 时 error 是失败原因。
    """
    model_name = model or config.ASR_MODEL_NAME

    audio = _denoise(audio, samplerate)
    wav = _to_wav_bytes(audio, samplerate)
    audio_b64 = base64.b64encode(wav.read()).decode("utf-8")

    task_id = str(uuid.uuid4())
    _hdr = {
        "X-Api-Key": config.ASR_API_KEY,
        "X-Api-Resource-Id": config.ASR_RESOURCE_ID,
        "X-Api-Sequence": "-1",
        "Content-Type": "application/json",
    }
    body = {
        "audio": {
            "data": audio_b64,
            "format": "wav",
            "codec": "raw",
            "rate": samplerate,
            "bits": 16,
            "channel": 1,
        },
        "request": {"model_name": model_name},
    }

    def _fail(msg):
        return {"ok": False, "text": None, "model": model_name,
                "usage": None, "error": msg}

    # 1) 提交任务
    try:
        resp = _ASR_SESSION.post(config.ASR_BASE_URL + "/submit",
                             headers={**_hdr, "X-Api-Request-Id": task_id},
                             json=body, timeout=60)
    except requests.RequestException as e:
        return _fail(str(e))
    if resp.status_code != 200:
        return _fail(f"提交 HTTP {resp.status_code}: {resp.text[:300]}")
    code = resp.headers.get("X-Api-Status-Code", "")
    if code not in ("", "20000000"):
        return _fail(f"提交失败 code={code}: {resp.text[:200]}")

    # 2) 轮询查询结果
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            q = _ASR_SESSION.post(config.ASR_BASE_URL + "/query",
                              headers={**_hdr, "X-Api-Request-Id": task_id},
                              json={}, timeout=60)
        except requests.RequestException as e:
            return _fail(str(e))
        qcode = q.headers.get("X-Api-Status-Code", "")
        try:
            data = q.json()
        except ValueError:
            return _fail(f"查询返回非 JSON: {q.text[:200]}")
        text = ((data.get("result") or {}).get("text") or "").strip()
        if text:
            return {"ok": True, "text": text, "model": model_name,
                    "usage": None, "error": None}
        if qcode in ("20000000", "20000003"):
            # 任务已结束但识别为空（静音/无有效语音），不当作错误
            return {"ok": True, "text": "", "model": model_name,
                    "usage": None, "error": None}
        if qcode and not qcode.startswith("2"):
            return _fail(f"识别错误 code={qcode}: {q.text[:200]}")
        time.sleep(0.1)
    return _fail("识别超时：查询 15 秒内未出结果")


# 识别结果过滤：环境噪声、他人单字应和、电视背景音常被 ASR 识别成这类无意义碎片。
# 只做确定性过滤（去标点后过短 / 纯语气词），不依赖置信度取值范围，避免机器人对背景杂音自说自话。
_FILLER_WORDS = {
    "嗯", "啊", "哦", "呃", "噢", "唉", "哎", "喔", "诶", "嗨", "喂",
    "嗯嗯", "啊啊", "哦哦", "哈哈", "嘿嘿", "呵呵", "嘶", "啧", "呼",
}


def _is_valid_utterance(text):
    t = (text or "").strip()
    if not t:
        return False
    core = re.sub(r"[\s，。！？!?、,.:：;；…~～\"'“”‘’\-—]", "", t)
    if len(core) < config.ASR_MIN_TEXT_LEN:
        return False
    if core in _FILLER_WORDS:
        return False
    return True


class SpeechListener:
    """实时连续语音识别：后台线程采集声音 + VAD 判断起止 + 自动识别。

    用法：
        listener = SpeechListener()
        listener.start()
        ...主循环里轮询 listener.latest_text 显示...
        listener.stop()

    内部用两个线程：
    - 采集线程：sounddevice 回调里做能量检测（VAD），把一句话的音频拼起来，
      说完静音就把这段音频放进队列。
    - 识别线程：从队列取音频去调「转文字」接口（网络慢，不能阻塞采集线程）。
    识别结果写入 latest_text，供主循环读取。
    """

    def __init__(self, samplerate=None, threshold=None, silence_sec=None,
                 min_speech_sec=None, max_speech_sec=None, model=None):
        self.samplerate = samplerate or config.ASR_SAMPLE_RATE
        self._adaptive = threshold is None
        self.threshold = config.VAD_THRESHOLD if threshold is None else threshold
        self.silence_sec = (config.VAD_SILENCE_SEC if silence_sec is None
                            else silence_sec)
        self.min_speech_sec = (config.VAD_MIN_SPEECH_SEC if min_speech_sec is None
                               else min_speech_sec)
        self.max_speech_sec = (config.VAD_MAX_SPEECH_SEC if max_speech_sec is None
                               else max_speech_sec)
        self.model = model or config.ASR_MODEL_NAME

        # 双阈值滞回：说话中音量降到“持续阈值”仍算继续说话，避免句中轻声被当成静音切断
        self.hold_threshold = self.threshold * getattr(config, "VAD_HOLD_RATIO", 0.6)
        self.postroll_sec = getattr(config, "VAD_POSTROLL_SEC", 0.25)
        # 持续自适应底噪：静音段用慢速 EMA 跟踪环境噪声，动态抬高阈值，
        # 避免开机后环境变吵（空调/电视/走动声）被当成"说话"持续误触发。
        self._noise_rms = 0.0
        self._adapt_alpha = 0.06
        # 预滚动缓存：留最近约 PREROLL_SEC 的音频，语音触发时补回被阈值吞掉的首字
        self._preroll_max = max(1, int(round(
            getattr(config, "VAD_PREROLL_SEC", 0.3) * self.samplerate
            / int(self.samplerate * 0.1))))
        self._preroll = deque(maxlen=self._preroll_max)

        # 十字线：最新识别结果（主线程读）
        self.latest_text = ""
        self._text_lock = threading.Lock()

        # 录音/识别线程内部状态
        self._stream = None
        self._running = False
        self._tasks = deque()          # 待识别的音频任务
        self._pending_buf = []         # 正在采集的一句话
        self._speaking = False         # 当前是否处于说话中
        self._silence_accum = 0.0      # 静音累计时长
        self._speech_accum = 0.0       # 当前这句话已说话时长
        self._collect_lock = threading.Lock()
        self._suppressed = False       # 机器人正在播报时置 True，丢弃麦克风采集

        self._threads = []

    def start(self):
        import sounddevice as sd
        if self._running:
            return
        baseline = 0.0
        if self._adaptive:
            baseline = self._measure_baseline(sd)
            self.threshold = float(np.clip(
                baseline * 3.0,
                float(getattr(config, "VAD_THRESHOLD_MIN", 250)),
                float(getattr(config, "VAD_THRESHOLD_MAX", 1600))))
            self.hold_threshold = self.threshold * getattr(config, "VAD_HOLD_RATIO", 0.6)
            self._noise_rms = baseline
        self._running = True
        self._stream = sd.InputStream(
            samplerate=self.samplerate, channels=1, dtype="int16",
            blocksize=int(self.samplerate * 0.1),  # 100ms 一块，VAD 足够
            callback=self._audio_callback)
        self._stream.start()
        self._threads = [
            threading.Thread(target=self._recognize_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()
        logger.info("语音监听已启动（阈值=%.0f，底噪=%.0f，静音=%ss，模型=%s）",
                    self.threshold, baseline, self.silence_sec, self.model)

    def _measure_baseline(self, sd):
        """静音采样约 1 秒，估算环境底噪 RMS（取分块中位数，抗说话干扰）。"""
        chunks = []

        def _cb(indata, frames, time_info, status):
            chunks.append(indata.copy())

        try:
            stream = sd.InputStream(
                samplerate=self.samplerate, channels=1, dtype="int16",
                blocksize=int(self.samplerate * 0.1), callback=_cb)
            with stream:
                time.sleep(1.0)
        except Exception:
            return 0.0
        if not chunks:
            return 0.0
        rms_list = []
        for ch in chunks:
            mono = ch[:, 0] if ch.ndim > 1 else ch
            if len(mono):
                rms_list.append(float(np.sqrt(np.mean(mono.astype(np.float64) ** 2))))
        if not rms_list:
            return 0.0
        rms_list.sort()
        return rms_list[len(rms_list) // 2]

    def stop(self):
        self._running = False
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None
        logger.info("语音监听已停止")

    # ---------- 音频采集（sounddevice 回调线程） ----------
    def _audio_callback(self, indata, frames, time_info, status):
        if not self._running:
            return
        mono = indata[:, 0] if indata.ndim > 1 else indata
        loudness = float(np.sqrt(np.mean(mono.astype(np.float64) ** 2))) if len(mono) else 0.0
        block_sec = frames / self.samplerate

        with self._collect_lock:
            if self._suppressed:
                # 机器人正在播报：丢弃这段声音并清空 VAD 状态，避免把喇叭声当成用户说话
                self._pending_buf = []
                self._preroll.clear()
                self._speaking = False
                self._silence_accum = 0.0
                self._speech_accum = 0.0
                return

            # 始终滚动缓存最近几块（含本块），供语音触发时补回“被阈值吞掉的首字”
            self._preroll.append(mono.copy())

            # 持续自适应底噪：只在“未在说话”且当前块判定为静音时，用慢速 EMA 跟踪环境噪声，
            # 动态抬高/调低阈值。这样开机后环境变吵（空调/电视/走动声）或变安静，阈值都会
            # 跟着变，不会被稳态噪声长期误触发成“说话”。
            if (not self._speaking and self._noise_rms >= 0.0
                    and loudness < self.threshold):
                self._noise_rms = (self._noise_rms * (1.0 - self._adapt_alpha)
                                   + loudness * self._adapt_alpha)
                if self._adaptive:
                    self.threshold = float(np.clip(
                        self._noise_rms * 3.0,
                        float(getattr(config, "VAD_THRESHOLD_MIN", 250)),
                        float(getattr(config, "VAD_THRESHOLD_MAX", 1600))))
                    self.hold_threshold = (self.threshold
                                           * getattr(config, "VAD_HOLD_RATIO", 0.6))

            # 双阈值滞回：说话中用更低的持续阈值，避免句中轻声被当成静音切断
            active_threshold = (self.hold_threshold if self._speaking
                                else self.threshold)
            if loudness >= active_threshold:
                if not self._speaking:
                    # 刚起话：把触发前的 preroll 并进这句话开头，避免首字缺失
                    self._pending_buf.extend(list(self._preroll))
                    self._preroll.clear()
                    self._speaking = True
                    self._silence_accum = 0.0
                    self._speech_accum = block_sec
                    if self._speech_accum >= self.max_speech_sec:
                        self._flush_pending()
                else:
                    self._speaking = True
                    self._silence_accum = 0.0
                    self._speech_accum += block_sec
                    self._pending_buf.append(mono.copy())
                    # 说话太久强制切分，保证能尽快出结果
                    if self._speech_accum >= self.max_speech_sec:
                        self._flush_pending()
            else:
                if self._speaking:
                    # 说话间隙里的静音，继续累积这句话
                    self._pending_buf.append(mono.copy())
                    self._silence_accum += block_sec
                    if self._silence_accum >= self.silence_sec - _FLT_EPS:
                        self._flush_pending()
                # 没在说话时的静音不累积，但已留在 preroll 里备着

    def _flush_pending(self):
        """把当前这句话打包成一个识别任务，投入队列。"""
        speech_dur = self._speech_accum   # 有效说话时长（不含尾静音）
        if not self._pending_buf:
            self._speaking = False
            self._silence_accum = 0.0
            self._speech_accum = 0.0
            return
        audio = np.concatenate(self._pending_buf)
        self._pending_buf = []
        self._speaking = False
        self._silence_accum = 0.0
        self._speech_accum = 0.0
        if speech_dur < self.min_speech_sec:
            logger.debug("丢弃过短片段（说话 %.2fs）", speech_dur)
            return
        audio = self._trim_trailing_silence(audio)
        self._tasks.append(audio)

    def _trim_trailing_silence(self, audio):
        """裁掉句尾过长的静音，只保留 postroll_sec，让识别更聚焦在有声段上。"""
        block = int(self.samplerate * 0.1)
        n = len(audio)
        if n <= block:
            return audio
        thr = self.hold_threshold
        i = n
        while i - block >= 0:
            seg = audio[i - block:i]
            rms = float(np.sqrt(np.mean(seg.astype(np.float64) ** 2)))
            if rms >= thr:
                break
            i -= block
        if i >= n:
            return audio                      # 结尾就是有声块，无需裁
        keep = int(self.postroll_sec * self.samplerate)
        i = min(n, i + keep)
        return audio[:i]

    # ---------- 识别（后台线程） ----------
    def _recognize_loop(self):
        while self._running:
            # 取一个任务（无任务时短暂睡眠，让出 CPU）
            with self._collect_lock:
                if self._tasks:
                    audio = self._tasks.popleft()
                else:
                    audio = None
            if audio is None:
                time.sleep(0.05)
                continue

            try:
                t0 = time.time()
                result = transcribe(audio, self.samplerate, model=self.model)
                logger.info("[耗时] 语音识别 %.2fs", time.time() - t0)
            except Exception as e:
                logger.warning("识别异常: %s", e)
                continue
            if result["ok"]:
                if result["text"] and _is_valid_utterance(result["text"]):
                    with self._text_lock:
                        self.latest_text = result["text"]
                    logger.info("识别: %s", result["text"])
                elif result["text"]:
                    logger.info("已过滤噪声/无效语音: %r", result["text"])
                else:
                    logger.info("识别为空（可能是杂音）")
            else:
                logger.warning("识别失败: %s", result["error"])
        # 退出前把残留任务处理掉
        with self._collect_lock:
            while self._tasks:
                audio = self._tasks.popleft()
                try:
                    result = transcribe(audio, self.samplerate, model=self.model)
                except Exception as e:
                    logger.warning("识别异常: %s", e)
                    continue
                if result["ok"] and result["text"] and _is_valid_utterance(result["text"]):
                    with self._text_lock:
                        self.latest_text = result["text"]

    # ---------- 对外接口 ----------
    def set_suppressed(self, value):
        """机器人正在播报时置 True，丢弃采集到的声音（避免把 TTS 当作用户说话）。"""
        with self._collect_lock:
            self._suppressed = bool(value)

    def is_speaking(self):
        """用户当前是否正在说话（供主流程判断，避免主动事件抢话）。"""
        with self._collect_lock:
            return self._speaking

    def get_text(self):
        with self._text_lock:
            return self.latest_text

    def clear_text(self):
        with self._text_lock:
            self.latest_text = ""