# -*- coding: utf-8 -*-
"""独居女性照料 · 多模态感知交互系统（仅本地运行版）

把摄像头、姿态识别(视觉)、豆包语音(语音)、对话(LLM)、语音播报(TTS)
装配成一条完整链路。

本文件已去掉一切 http 转发 / 内网穿透 / 队友交付相关代码：
动作意图只写入本地 command.json，不向任何网络推送。

执行顺序（按优先级从高到低）：
    语音求助指令（任务模式） > 经期剧情问答 > 视觉痛苦姿态关怀 > 普通闲聊。

经期剧情：整套流程只能完整跑一轮，跑完置 period_remind_status=finished
并落盘，之后任何对话都禁止再提及/询问经期相关内容。
"""

import json
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np

import config
import llm_chat
from pose_estimator import PoseEstimator
from posture import PostureClassifier
from speech import SpeechListener
from tts import Speaker

CAMERA_INDEX = 0                      # 摄像头设备号，换新摄像头只改这个数字
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
COMMAND_JSON_PATH = os.path.join(_BASE_DIR, "command.json")        # 动作意图（仅本地）
_PERIOD_STATE_FILE = os.path.join(_BASE_DIR, "period_remind_state.json")  # 经期 finished 落盘

# ===== 经期剧情固定话术 =====
SAY_ASK_UNWELL = "你是不是不舒服？"
SAY_ASK_PERIOD = "根据最近和您的对话推测您目前正处于经期阶段，请问我的推测准确吗？"
SAY_PERIOD_ADVICE = ("经期要注意腹部保暖，尽量别碰生冷食物，多喝温水，多休息，"
                     "别太劳累；如果肚子明显疼痛，可以及时告诉我。")
SAY_ASK_PAINKILLER = "需要我帮你拿止痛药吗？"
SAY_PAINKILLER_YES = "好的，我这就为您拿止痛药。"
SAY_PAINKILLER_NO = "好，如果之后有需要可以再告诉我。"
SAY_PERIOD_DENY = "了解，如果身体难受可以和我说。"

# ===== 痛苦姿态关怀话术（经期 finished 之后）=====
SAY_PAIN_CARE = "看到你好像不太舒服，你是哪里不舒服吗？"

# ===== 语音求助指令固定话术 =====
SAY_CONFIRM_WATER = "好的，我这就为您拿水杯。"
SAY_CONFIRM_TISSUE = "收到，我马上帮你拿纸巾过来。"
SAY_CONFIRM_PAINKILLER = "好的，我这就为您拿止痛药。"

SAY_CHAT_FALLBACK = "嗯，我在听呢，你想说什么呀？"
SAY_PERIOD_PROMPT = "抱歉，我听不太明白。请直接回答：是的，或者不是。"

# ===== 持续时间 / 冷却 =====
PERIOD_BELLY_HOLD_SEC = 2.0    # 捂肚子持续≥2秒才开启经期剧情（步骤1）
PAIN_POSTURE_HOLD_SEC = 2.0    # 痛苦姿态持续≥2秒才触发关怀（短暂动作不触发）
PAIN_CARE_COOLDOWN_SEC = 30.0  # 痛苦姿态关怀冷却，避免反复刷屏

# ===== 关键词 =====
# 经期剧情第3步确认（“推测准确吗？”）
_CONSENT_WORDS = ("是的", "对的", "没错", "准确", "挺准", "蛮准", "差不多",
                  "需要", "可以", "好的", "要", "对", "是", "准", "行", "好")
_PERIOD_DENY_WORDS = ("不准确", "不准", "不太准", "没那么准", "不是", "不对", "错了", "错")
# 第4步“需要拿止痛药吗？”的否定
_PAINKILLER_DENY_WORDS = ("不需要", "不用", "不要", "没必要", "不用了", "算了", "不了", "没事")
# 第1步“有什么不舒服吗？”的否定（否认不舒服，不进入剧情）
_UNWELL_DENY_WORDS = ("没有", "没事", "还好", "没什么", "好了", "不难受", "不疼", "不痛")
# 第2步“肚子不舒服”判定
_STOMACH_WORDS = ("肚子", "腹部", "小腹", "胃", "痛经", "经痛", "姨妈", "月经", "腹痛")


def write_command(action):
    """产生机械臂动作意图：只写入本地 command.json，不联网、不发送。"""
    payload = {"action": action, "ts": datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
    try:
        with open(COMMAND_JSON_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        print("[本地指令→command.json]", json.dumps(payload, ensure_ascii=False))
    except Exception as e:
        print("[写 command.json 失败]", e)


def is_pain_posture(posture, actions):
    """五种痛苦姿态（全部需持续≥2秒，依靠关键点组合，不完全依赖姿态标签）。

    1. 蹲姿双手捂肚子（即便被误判为坐姿，靠捂腹+持续仍触发）
    2. 双手抱住头部
    3. 身体蜷缩 / 长时间蹲地
    4. 平躺 + 双手捂肚子
    5. 平躺 + 来回辗转翻滚
    """
    belly = ("hand_on_belly" in actions or "belly_hold" in actions)
    if belly and posture in ("squatting", "sitting", "kneeling"):
        return True
    if "hand_on_head" in actions:
        return True
    if "curled_up" in actions or posture == "squatting":
        return True
    if posture == "lying" and belly:
        return True
    if posture == "lying" and "writhing" in actions:
        return True
    return False


def match_assistance(text):
    """语音求助指令（高优先级）：直接响应任务，禁止回溯经期历史话题。

    返回 (say, action)，未命中返回 (None, None)。
    """
    t = (text or "").strip()
    if any(w in t for w in ("止痛", "止疼", "布洛芬")):
        return (SAY_CONFIRM_PAINKILLER, "fetch_painkiller")
    if any(w in t for w in ("纸巾", "抽纸", "餐巾纸")):
        return (SAY_CONFIRM_TISSUE, "fetch_tissue")
    if any(w in t for w in ("水杯", "拿水", "喝水", "杯水", "递水", "倒水", "倒杯水")):
        return (SAY_CONFIRM_WATER, "fetch_watercup")
    return (None, None)


class PeriodFlow:
    """经期交互剧情状态机：完整流程只能跑一轮，finished 后永久关闭（落盘）。

    阶段流转：
        idle ──视觉持续捂肚子──▶ ask_unwell ──用户说“肚子不舒服”──▶ ask_period
        ask_period ──“是的”──▶ ask_painkiller ──“是的/不用”──▶ finished
        ask_period ──“不是”──▶ finished
    """

    STAGE_IDLE = "idle"
    STAGE_ASK_UNWELL = "ask_unwell"
    STAGE_ASK_PERIOD = "ask_period"
    STAGE_ASK_PAINKILLER = "ask_painkiller"

    def __init__(self):
        self.status = self._load_status()   # "pending" / "finished"
        self.stage = self.STAGE_IDLE

    # ---- 状态落盘：跑完一次即永久关闭，重启也不复现 ----
    def _load_status(self):
        try:
            with open(_PERIOD_STATE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data.get("period_remind_status", "pending")
        except (OSError, ValueError, json.JSONDecodeError):
            return "pending"

    def _save_finished(self):
        self.status = "finished"
        self.stage = self.STAGE_IDLE
        try:
            with open(_PERIOD_STATE_FILE, "w", encoding="utf-8") as f:
                json.dump({"period_remind_status": "finished"}, f, ensure_ascii=False, indent=2)
            print("[经期剧情] 已置 finished，经期分支永久关闭")
        except OSError as e:
            print("[经期剧情] 落盘失败:", e)

    @property
    def finished(self):
        return self.status == "finished"

    @property
    def active(self):
        """是否正处在剧情提问中（需等用户回答）。"""
        return self.stage != self.STAGE_IDLE

    def try_start(self, belly_sustained):
        """视觉持续捂肚子 → 开启剧情，主动发问。返回要播报的话，未触发返回 None。"""
        if self.finished or self.active or not belly_sustained:
            return None
        self.stage = self.STAGE_ASK_UNWELL
        return SAY_ASK_UNWELL

    def on_text(self, text):
        """剧情提问中处理用户回答。

        返回 (say, action) 表示由剧情应答；None 表示这句话不命中剧情，交给上层处理。
        """
        if self.finished or not self.active or not text:
            return None
        t = (text or "").strip()
        if not t:
            return None

        if self.stage == self.STAGE_ASK_UNWELL:
            # 先判肚子不舒服，再判否认（避免“我不是很舒服”被“不是”误伤）
            if any(w in t for w in _STOMACH_WORDS):
                self.stage = self.STAGE_ASK_PERIOD
                return (SAY_ASK_PERIOD, None)
            if any(w in t for w in _UNWELL_DENY_WORDS):
                self.stage = self.STAGE_IDLE   # 否认不舒服：不进剧情，也不 finished
                return None
            return None

        if self.stage == self.STAGE_ASK_PERIOD:
            if any(w in t for w in _PERIOD_DENY_WORDS):
                self._save_finished()
                return (SAY_PERIOD_DENY, None)
            if any(w in t for w in _CONSENT_WORDS):
                # 第一步：注意事项；第二步：紧接着提问（拼接一次性播报，TTS 会自然断句）
                self.stage = self.STAGE_ASK_PAINKILLER
                return (SAY_PERIOD_ADVICE + SAY_ASK_PAINKILLER, None)
            return None

        if self.stage == self.STAGE_ASK_PAINKILLER:
            if any(w in t for w in _PAINKILLER_DENY_WORDS):
                self._save_finished()
                return (SAY_PAINKILLER_NO, None)
            if any(w in t for w in _CONSENT_WORDS) or any(w in t for w in ("止痛", "止疼", "布洛芬")):
                self._save_finished()
                return (SAY_PAINKILLER_YES, "fetch_painkiller")
            return None

        return None


def _chat_text(text, history=None):
    """普通闲聊：读完整对话历史，像豆包一样多轮聊天。finished 后禁止再提经期。"""
    sys_prompt = config.LLM_CHAT_SYSTEM_PROMPT.format(
        now_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    sys_prompt += "\n额外红线：绝对不要提及或询问月经、生理期、痛经等相关内容。"
    r = llm_chat.chat(text, system_prompt=sys_prompt, history=history)
    if r.get("ok") and r.get("text"):
        return r["text"]
    print("[闲聊·大模型无返回]", r.get("error"))
    return ""


# ---------------------- 中文叠加（OpenCV 演示窗口） ----------------------
_C_WHITE = (255, 255, 255)
_C_GOLD = (255, 210, 120)
_C_GREEN = (95, 255, 160)

try:
    from PIL import Image, ImageDraw, ImageFont
    _CN_FONT_PATH = None

    def _cn_font(size):
        global _CN_FONT_PATH
        if _CN_FONT_PATH is None:
            _CN_FONT_PATH = ""
            for fp in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf"):
                if os.path.exists(fp):
                    _CN_FONT_PATH = fp
                    break
        try:
            return ImageFont.truetype(_CN_FONT_PATH, size) if _CN_FONT_PATH \
                else ImageFont.load_default()
        except Exception:
            return ImageFont.load_default()

    def _wrap_text(text, max_units):
        lines = []
        cur = ""
        width = 0
        for ch in text:
            cw = 2 if ord(ch) > 0x2E80 else 1
            if width + cw > max_units and cur:
                lines.append(cur)
                cur = ch
                width = cw
            else:
                cur += ch
                width += cw
        if cur:
            lines.append(cur)
        return lines or [""]

    def _clip_lines(text, max_units, max_lines):
        lines = _wrap_text(text, max_units)
        if len(lines) > max_lines:
            lines = lines[:max_lines]
            lines[-1] = lines[-1][:-1] + "…" if len(lines[-1]) > 1 else "…"
        return lines

    def put_cn(frame, items):
        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(img)
        h = frame.shape[0]
        rows = []
        for item in items:
            text, rgb = item[0], item[1]
            cap = item[2] if len(item) > 2 else None
            lines = _clip_lines(text, 34, cap) if cap else _wrap_text(text, 34)
            for ln in lines:
                rows.append((ln, rgb))
        if not rows:
            return frame
        y = h - 28 * len(rows) - 14
        font = _cn_font(20)
        for text, rgb in rows:
            w = int(d.textlength(text, font=font)) + 18
            d.rectangle([10, y - 4, 12 + w, y + 26], fill=(0, 0, 0))
            d.text((20, y), text, font=font, fill=rgb)
            y += 28
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
except Exception:
    def put_cn(frame, items):
        return frame


def _posture_cn(p):
    return {"standing": "站立", "sitting": "坐姿", "squatting": "蹲姿",
            "kneeling": "跪姿", "lying": "躺卧", "unknown": "未知"}.get(p, p)


def _action_cn(actions):
    m = {
        "hand_on_belly": "捂肚子", "hand_on_head": "抱头", "curled_up": "蜷缩",
        "belly_hold": "双手捂腹", "fallen": "倒地", "writhing": "打滚", "pacing": "踱步",
    }
    return "、".join(m.get(a, a) for a in actions) if actions else "无"


def main():
    speaker = Speaker()
    listener = SpeechListener()
    try:
        listener.start()
    except Exception as e:
        print("[语音] 启动失败（本次禁用语音识别）:", e)
        listener = None

    pose_est = PoseEstimator()
    cls = PostureClassifier()
    period = PeriodFlow()
    print("[经期剧情] 当前状态 =", period.status,
          "（finished=已跑完，pending=待触发；删除 period_remind_state.json 可重置）")

    # 普通闲聊的短期记忆：保留最近几轮，让上下文连贯
    chat_history = deque(maxlen=8)
    _chat_lock = threading.Lock()

    # 大模型后台线程：把慢的豆包请求从摄像头主循环里隔开，避免画面卡顿
    llm_q = queue.Queue()

    def _llm_worker():
        while True:
            job = llm_q.get()
            if job is None:
                break
            kind, payload = job
            try:
                if kind == "chat":
                    user_text = payload["text"]
                    with _chat_lock:
                        history = list(chat_history)
                    reply = _chat_text(user_text, history)
                    if not reply:
                        reply = SAY_CHAT_FALLBACK
                    with _chat_lock:
                        chat_history.append({"role": "user", "content": user_text})
                        chat_history.append({"role": "assistant", "content": reply})
                    speaker.say(reply)
            except Exception as e:
                print("[闲聊处理失败]", e)

    threading.Thread(target=_llm_worker, daemon=True).start()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[摄像头] 打开失败，请检查 CAMERA_INDEX（当前=", CAMERA_INDEX, "）")
        return
    print("[摄像头] 已打开（CAMERA_INDEX=", CAMERA_INDEX, "）")
    print("[提示] 按 A 模拟捂肚子触发经期 / T 拿纸巾指令 / W 拿水杯指令 / Q 退出")

    last_speech = ""       # 最近识别到的用户说话（绿色回显）
    belly_start = None     # 捂肚子持续计时（经期入口）
    pain_start = None      # 痛苦姿态持续计时（关怀）
    pain_care_last = 0.0   # 上次痛苦关怀播报时刻（冷却）
    running = True

    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        frame = cv2.flip(frame, 1)   # 镜像显示更自然

        # 1) 视觉：姿态识别 + 动作分类
        posture, actions, metrics = "unknown", [], {}
        try:
            lm = pose_est.detect(frame)
            if lm is not None:
                posture, actions, metrics = cls.classify(lm)
        except Exception as e:
            print("[视觉]", e)

        now = time.time()

        # 捂肚子持续计时（经期剧情入口，任意姿态；finished 后不再用）
        belly = ("hand_on_belly" in actions or "belly_hold" in actions)
        if belly:
            if belly_start is None:
                belly_start = now
            belly_sustained = (now - belly_start) >= PERIOD_BELLY_HOLD_SEC
        else:
            belly_start = None
            belly_sustained = False

        # 痛苦姿态持续计时（仅 finished 后生效）
        pain = period.finished and is_pain_posture(posture, actions)
        if pain:
            if pain_start is None:
                pain_start = now
        else:
            pain_start = None

        # 2) 语音：提取用户已说完的话；机器人出声时抑制麦克风防止回声自激
        text = ""
        user_speaking = False
        if listener is not None:
            listener.set_suppressed(speaker.is_speaking())
            user_speaking = listener.is_speaking()
            text = listener.get_text()

        if text and text != last_speech:
            last_speech = text   # 回显用户新说的一句话（绿色）

        # 3) 决策：语音（任务/剧情/闲聊）优先；无语音时再处理主动视觉触发
        if text and not speaker.is_speaking():
            t = text.strip()
            handled = False

            # ① 经期剧情提问中：优先处理剧情应答（“是的/不是/肚子不舒服”）
            if period.active:
                res = period.on_text(t)
                if res is not None:
                    say, action = res
                    if say:
                        speaker.say(say)
                    if action:
                        write_command(action)
                    handled = True

            # ② 语音求助指令（最高优先级；收到求助直接响应，禁止回溯经期历史）
            if not handled:
                say, action = match_assistance(t)
                if action:
                    speaker.say(say)
                    write_command(action)
                    handled = True

            # ③ 经期剧情中答非所问：绝不落给 LLM 闲聊，固定话术引导按剧本回答
            if not handled and period.active:
                speaker.say(SAY_PERIOD_PROMPT)
                handled = True

            # ④ 普通闲聊：读完整对话历史，多轮聊天
            if not handled:
                llm_q.put(("chat", {"text": t}))

            if listener is not None:
                listener.clear_text()

        # 4) 主动视觉触发（机器人没在播报、用户没在说话时才问，避免抢话）
        elif not speaker.is_speaking() and not user_speaking:
            say = period.try_start(belly_sustained)
            if say:
                speaker.say(say)
            elif (pain and pain_start is not None
                  and (now - pain_start) >= PAIN_POSTURE_HOLD_SEC
                  and (now - pain_care_last) >= PAIN_CARE_COOLDOWN_SEC):
                pain_care_last = now
                speaker.say(SAY_PAIN_CARE)

        # 5) 键盘兜底
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            running = False
            break
        elif key in (ord('a'), ord('A')):      # 模拟捂肚子触发经期剧情
            say = period.try_start(True)
            if say:
                speaker.say(say)
            else:
                print("[键盘] 经期已 finished，不再触发剧情")
        elif key in (ord('t'), ord('T')):      # 拿纸巾指令
            write_command("fetch_tissue")
        elif key in (ord('w'), ord('W')):      # 拿水杯指令
            write_command("fetch_watercup")

        # 6) 演示窗口叠加状态 + 显示
        assist_action = match_assistance(text)[1] if text else None
        if period.active:
            mode_label, status = "经期剧情模式", "经期问答中"
        elif period.finished and is_pain_posture(posture, actions):
            mode_label, status = "任务模式", "检测到不适姿态"
        elif assist_action:
            mode_label, status = "任务模式", "响应求助指令"
        else:
            mode_label, status = "普通闲聊模式", "待机"

        items = [
            ("模式: " + mode_label, _C_WHITE),
            ("状态: " + status, _C_WHITE),
            ("姿态: " + _posture_cn(posture) + "   动作: " + _action_cn(actions), _C_WHITE),
        ]
        if speaker.last_text:
            items.append(("回复: " + speaker.last_text, _C_GOLD, 3))
        if last_speech:
            items.append(("说话: " + last_speech, _C_GREEN, 2))
        frame = put_cn(frame, items)
        cv2.imshow("CareSystem (按 q 退出)", frame)

    cap.release()
    cv2.destroyAllWindows()
    if listener is not None:
        listener.stop()
    llm_q.put(None)
    print("已退出。")


if __name__ == "__main__":
    main()