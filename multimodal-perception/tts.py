"""本地语音播报（TTS）：把机器人要说的文字用自然的神经语音读出来。

语音链（按优先级，逐级自动兜底）：
1. 火山引擎「语音合成大模型」（柔美女友2.0，最自然；需在 config.py 填 TTS_VOLCANO_API_KEY）
   → 合成 MP3，用 Windows MCI 播放；
2. edge-tts（微软知性女声 zh-CN-XiaoyiNeural，免费、有自然断句）；
3. pyttsx3 / SAPI（Windows 自带语音，仅最后兜底）。

断句预处理（最关键）：交给任何 TTS 前，先清洗文本——去掉 markdown/换行/特殊符号、
折叠空白、自动补标点、把过长又没标点的长串文字拆短。因为 TTS 完全靠标点定停顿，
没标点就会乱断句、读得很怪。

对外接口：Speaker().say(text) —— 文本进队列，后台线程依次朗读，不阻塞画面。
"""

import asyncio
import base64
import ctypes
import json
import logging
import os
import queue
import re
import subprocess
import tempfile
import threading
import time
import uuid

import requests

import config

logger = logging.getLogger("tts")
if not logger.handlers:
    _h = logging.StreamHandler()
    _h.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
    logger.addHandler(_h)
    logger.setLevel(logging.INFO)

TTS_PROVIDER = getattr(config, "TTS_PROVIDER", "auto")
TTS_VOLCANO_VOICE = getattr(config, "TTS_VOLCANO_VOICE", "zh_female_roumeinvyou_uranus_bigtts")
TTS_VOLCANO_API_KEY = getattr(config, "TTS_VOLCANO_API_KEY", "")
TTS_VOLCANO_RESOURCE_ID = getattr(config, "TTS_VOLCANO_RESOURCE_ID", "seed-tts-2.0")
TTS_VOLCANO_SPEECH_RATE = getattr(config, "TTS_VOLCANO_SPEECH_RATE", -10)
TTS_VOLCANO_URL = "https://openspeech.bytedance.com/api/v3/tts/unidirectional"
TTS_EDGE_VOICE = getattr(config, "TTS_EDGE_VOICE", "zh-CN-XiaoyiNeural")
TTS_EDGE_RATE = getattr(config, "TTS_EDGE_RATE", "-8%")
TTS_MAX_SENT_LEN = getattr(config, "TTS_MAX_SENT_LEN", 16)


# ============ 断句预处理 ============

_MARKDOWN_RE = re.compile(r"[#*`~>|_\-=\[\](){}【】〈〉<>]")
_SENT_END = "。！？!?；;…"
# 常见口语连词，读到这里天然会停顿；在它前面补逗号能帮助 TTS 断句。
# 只收录"句首型"连词；疑问助词（是不是/要不要/能不能/有没有）常跟在主语后，会误加逗号，故不收。
_CONNECTIVE_RE = re.compile(
    r"(?<!^)(?<![，,。！？!?；;：:…~～\s])"
    r"(但是|可是|不过|然后|所以|因为|如果|而且|或者|还是|其实|还有|就是|比如|真的|这样)"
)


def clean_for_tts(text):
    """TTS 前清洗：去 markdown/特殊符号 → 换行转句间停顿 → 折叠空白 → 补自然停顿 → 补句末标点。

    只做「朗读友好」的微调，不改变语义；屏幕显示与语音共用同一份内容。
    """
    if not text:
        return ""
    t = str(text)
    t = t.replace("\u3000", " ")
    # 换行当句间停顿：上一句没有句末标点时补个句号，避免两句话糊在一起
    t = re.sub(r"(?<![。！？!?；;…])(?:[ \t]*[\r\n]+[ \t]*)+", "。", t)
    # 去 markdown 与特殊符号（保留正常中英文标点）
    t = _MARKDOWN_RE.sub("", t)
    # 折叠空白
    t = re.sub(r"[ \t]+", " ", t).strip()
    # 去掉紧贴标点的多余空格
    t = re.sub(r"\s*([，。！？!?；;…、])\s*", r"\1", t)
    # 在口语连词前补逗号，制造自然停顿（行首/标点后不补）
    t = _CONNECTIVE_RE.sub(r"，\1", t)
    # 去首尾多余标点
    t = t.strip(" \t,，;；:：")
    if not t:
        return ""
    t = _hard_chunk(t)
    if t[-1] not in _SENT_END:
        t += "。"
    return t


def _hard_chunk(t):
    """最后的兜底：仍有很长又完全没停顿时，才按阈值强制拆短（很少触发）。"""
    max_len = TTS_MAX_SENT_LEN
    segs = re.split(r"(?<=[。！？!?；;…])", t)
    out = []
    for seg in segs:
        if not seg:
            continue
        if len(seg) > max_len * 2 + 4 and not any(c in seg for c in "，,、：:"):
            seg = "，".join([seg[i:i + max_len] for i in range(0, len(seg), max_len)])
        out.append(seg)
    return "".join(out)


def _split_sentences(text):
    """按句末标点切分（保留标点），用于句与句之间加"呼吸停顿"。"""
    parts = re.split(r"(?<=[。！？!?；;…])", text)
    return [p.strip() for p in parts if p and p.strip()]


# ============ 音频播放 ============

def _mci(cmd):
    buf = ctypes.create_unicode_buffer(1024)
    ret = ctypes.windll.winmm.mciSendStringW(cmd, buf, 1024, None)
    if ret:
        err = ctypes.create_unicode_buffer(256)
        ctypes.windll.winmm.mciGetErrorStringW(ret, err, 256)
        raise RuntimeError(f"MCI {ret}: {err.value}")
    return buf.value


def _play_mp3_blocking(path, alias="tts"):
    """用 MCI 播放 MP3，阻塞到播放结束（结束后自行 close）。"""
    _mci(f'open "{path}" type mpegvideo alias {alias}')
    try:
        _mci(f"play {alias}")
        time.sleep(0.1)
        while True:
            try:
                mode = _mci(f"status {alias} mode").strip()
            except RuntimeError:
                break
            if mode != "playing":
                break
            time.sleep(0.1)
    finally:
        try:
            _mci(f"close {alias}")
        except Exception:
            pass


def _play_bytes_blocking(mp3_bytes):
    fd, path = tempfile.mkstemp(suffix=".mp3")
    os.close(fd)
    try:
        with open(path, "wb") as f:
            f.write(mp3_bytes)
        _play_mp3_blocking(path)
    finally:
        try:
            os.remove(path)
        except Exception:
            pass


# ============ edge-tts 兜底 ============

def _edge_available():
    try:
        import edge_tts  # noqa: F401
        return True
    except Exception:
        return False


def _edge_synth(sentence):
    """合成单句，返回临时 mp3 路径（阻塞网络请求，带 30s 超时防凌晨网络卡死）。"""
    import edge_tts
    c = edge_tts.Communicate(sentence, TTS_EDGE_VOICE, rate=TTS_EDGE_RATE)
    fd, path = tempfile.mkstemp(suffix=".mp3")
    os.close(fd)
    try:
        asyncio.run(asyncio.wait_for(c.save(path), timeout=30))
    except Exception:
        try:
            os.remove(path)
        except Exception:
            pass
        raise
    return path


def _edge_speak(text):
    """edge-tts 合成并逐句播放（阻塞）。

    优化：先合成并播放第一句（缩短"听到声音"前的等待），同时用后台线程预合成
    剩余句子，边播边合成，句与句之间不再卡在网络请求上。
    """
    sentences = _split_sentences(clean_for_tts(text))
    if not sentences:
        return
    pause = getattr(config, "TTS_SENTENCE_PAUSE_SEC", 0.15)
    paths = []
    try:
        _t0 = time.time()
        paths.append(_edge_synth(sentences[0]))
        logger.info("[耗时] TTS 首句合成 %.2fs", time.time() - _t0)

        if len(sentences) > 1:
            syn_q = queue.Queue()

            def _producer():
                try:
                    for s in sentences[1:]:
                        p = _edge_synth(s)
                        paths.append(p)
                        syn_q.put(p)
                finally:
                    syn_q.put(None)  # 结束哨兵

            threading.Thread(target=_producer, daemon=True).start()

        _play_mp3_blocking(paths[0])
        if len(sentences) > 1:
            for i in range(1, len(sentences)):
                p = syn_q.get()
                if p is None:
                    break
                _play_mp3_blocking(p)
                if pause > 0 and i < len(sentences) - 1:
                    time.sleep(pause)
    finally:
        for path in paths:
            try:
                os.remove(path)
            except Exception:
                pass


# ============ 火山引擎「语音合成大模型」 ============

def _volcano_available():
    return bool(TTS_VOLCANO_API_KEY)


def _volcano_speak(text):
    """火山引擎「语音合成大模型」合成并播放（阻塞）。失败抛异常，由上层兜底到 edge-tts。"""
    if not TTS_VOLCANO_API_KEY:
        raise RuntimeError("未配置 TTS_VOLCANO_API_KEY")
    headers = {
        "X-Api-Key": TTS_VOLCANO_API_KEY,
        "X-Api-Resource-Id": TTS_VOLCANO_RESOURCE_ID,
        "X-Api-Request-Id": str(uuid.uuid4()),
        "Content-Type": "application/json",
        "Connection": "keep-alive",
    }
    audio_params = {"format": "mp3", "sample_rate": 24000}
    if TTS_VOLCANO_SPEECH_RATE:
        audio_params["speech_rate"] = int(TTS_VOLCANO_SPEECH_RATE)
    body = {
        "req_params": {
            "text": clean_for_tts(text),
            "speaker": TTS_VOLCANO_VOICE,
            "audio_params": audio_params,
        }
    }
    _t0 = time.time()
    resp = requests.post(TTS_VOLCANO_URL, headers=headers, json=body, timeout=60)
    logger.info("[耗时] 火山TTS 合成 %.2fs", time.time() - _t0)
    if resp.status_code != 200:
        raise RuntimeError(f"火山 TTS HTTP {resp.status_code}: {resp.text[:200]}")
    raw = _extract_volcano_audio(resp)
    if not raw:
        raise RuntimeError("火山 TTS 返回音频为空")
    _play_bytes_blocking(raw)


def _check_volcano(obj):
    code = obj.get("code")
    if code not in (0, None):
        raise RuntimeError(f"火山 TTS code={code} msg={obj.get('message')}")


def _extract_volcano_audio(resp):
    """解析火山 TTS 响应：可能是单个 JSON、多段 JSON（流式）或纯 base64，返回 MP3 字节。"""
    text = (resp.text or "").strip()
    if not text:
        return b""
    if not text.startswith("{"):
        try:
            return base64.b64decode(text)
        except Exception:
            return b""

    def _b64(s):
        try:
            return base64.b64decode(s)
        except Exception:
            return b""

    # 常见：整段一个 JSON
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        obj = None
    if isinstance(obj, dict):
        _check_volcano(obj)
        return _b64(obj.get("data") or "")

    # 流式：多段 JSON 逐行
    parts = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            _check_volcano(obj)
            if obj.get("data"):
                parts.append(obj["data"])
    return _b64("".join(parts))


# ============ 播报器 ============

class Speaker:
    """非阻塞语音播报器：say(text) 丢进队列，后台线程依次朗读（不重叠）。"""

    def __init__(self, enabled=None):
        self.enabled = config.TTS_ENABLED if enabled is None else enabled
        self._q = queue.Queue()
        self._busy = False       # 当前是否正在朗读（供回声抑制判断）
        self._engine = None      # pyttsx3 引擎（最后兜底）
        self._mode = "sapi"      # volcano / edge / pyttsx3 / sapi
        self._last_text = ""     # 最近/正在朗读的原文（供主循环回显金色回复）
        if self.enabled:
            self._init_engine()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _init_engine(self):
        # 1) 火山引擎（优先）
        if TTS_PROVIDER in ("auto", "volcano") and _volcano_available():
            self._mode = "volcano"
            logger.info("TTS 已启用（火山引擎语音合成大模型：%s，语速 %s）",
                        TTS_VOLCANO_VOICE, TTS_VOLCANO_SPEECH_RATE)
            return
        # 2) edge-tts 免费神经语音
        if TTS_PROVIDER in ("auto", "edge") and _edge_available():
            self._mode = "edge"
            logger.info("TTS 已启用（edge-tts：%s，语速 %s）", TTS_EDGE_VOICE, TTS_EDGE_RATE)
            return
        # 3) pyttsx3
        try:
            import pyttsx3
            self._engine = pyttsx3.init()
            self._engine.setProperty("rate", config.TTS_VOICE_RATE)
            self._mode = "pyttsx3"
            logger.info("TTS 已启用（pyttsx3）")
            return
        except Exception as e:
            logger.info("pyttsx3 不可用（%s），回退到 PowerShell SAPI", e)
        self._mode = "sapi"

    def say(self, text):
        if not self.enabled:
            return
        text = (text or "").strip()
        if text:
            self._q.put((text, None))

    @property
    def last_text(self):
        """最近一次（或正在）朗读的原文，供主循环在画面上金色回显。"""
        return self._last_text

    def say_wait(self, text):
        """朗读并阻塞到读完。用于保证"画面文字"和"语音"严格对齐。"""
        if not self.enabled:
            return
        text = (text or "").strip()
        if not text:
            return
        ev = threading.Event()
        self._q.put((text, ev))
        ev.wait()

    def is_speaking(self):
        """是否有文本正在朗读或排队待读（供语音回声抑制：机器人出声时丢麦克风）。"""
        return self._busy or not self._q.empty()

    def _run(self):
        while True:
            item = self._q.get()
            if isinstance(item, tuple):
                text, ev = item
            else:
                text, ev = item, None
            self._last_text = text          # 回显用（与朗读内容一致）
            self._busy = True
            try:
                self._speak_text(text)
            except Exception as e:
                logger.warning("TTS 朗读失败: %s", e)
                try:
                    self._speak_text(text, fallback=True)
                except Exception as e2:
                    logger.warning("TTS 兜底也失败: %s", e2)
            finally:
                self._busy = False
                if ev is not None:
                    ev.set()

    def _speak_text(self, text, fallback=False):
        mode = self._mode
        if fallback:
            if mode == "volcano":
                mode = "edge" if _edge_available() else ("pyttsx3" if self._engine else "sapi")
            elif mode == "edge":
                mode = "pyttsx3" if self._engine else "sapi"
            else:
                mode = "sapi"
        if mode == "volcano":
            _volcano_speak(text)
        elif mode == "edge":
            _edge_speak(text)
        elif mode == "pyttsx3" and self._engine is not None:
            self._engine.say(clean_for_tts(text))
            self._engine.runAndWait()
        else:
            self._speak_sapi(clean_for_tts(text))

    @staticmethod
    def _speak_sapi(text):
        """SAPI 兜底：把文本嵌进脚本 base64 编码，阻塞读完再返回。"""
        safe = text.replace("'", "''")
        ps = (
            "Add-Type -AssemblyName System.Speech;"
            "$s=New-Object System.Speech.Synthesis.SpeechSynthesizer;"
            f"$s.Speak('{safe}')"
        )
        encoded = base64.b64encode(ps.encode("utf-16-le")).decode("ascii")
        try:
            subprocess.run(
                ["powershell", "-NoProfile", "-NonInteractive",
                 "-EncodedCommand", encoded],
                capture_output=True, timeout=60,
            )
        except Exception as e:
            logger.warning("PowerShell SAPI 失败: %s", e)