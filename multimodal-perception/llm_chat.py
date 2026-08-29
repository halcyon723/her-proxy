"""LLM 对话模块：把识别出的文字送给豆包大模型，得到答复/建议。

与语音识别衔接：ASR 转文字 → LLM 理解并回答。
输出：{ok, text, error}，ok=True 时 text 是豆包的答复。
"""

import time

import requests

import config

# 复用 HTTP 连接，降低后续轮次 TLS 握手延迟
_SESSION = requests.Session()


def _now_time_str():
    """返回本机当前时间字符串，供大模型做时间相关问答。"""
    return time.strftime("%Y-%m-%d %H:%M:%S")


def chat(user_text, model=None, system_prompt=None,
         now_time=None, vision_result=None, history=None):
    """送一句话给豆包，得到答复。

    now_time / vision_result 会自动填充进系统 Prompt 里的占位符，
    让模型知道"现在几点"和"摄像头看到了什么"。
    history 可选：上一轮的 [{"role":"user"/"assistant","content":...}] 对话，用于上下文记忆。
    """
    t0 = time.time()
    model = model or config.LLM_MODEL
    if system_prompt is None:
        system_prompt = config.LLM_SYSTEM_PROMPT.format(
            now_time=now_time or _now_time_str(),
            vision_pose_status=vision_result or "用户姿态正常，无异常动作",
        )

    messages = [{"role": "system", "content": system_prompt}]
    for m in (history or []):
        if isinstance(m, dict) and m.get("role") in ("user", "assistant") and m.get("content"):
            messages.append({"role": m["role"], "content": m["content"]})
    messages.append({"role": "user", "content": user_text})

    try:
        resp = _SESSION.post(
            f"{config.LLM_BASE_URL}/chat/completions",
            headers={"Authorization": f"Bearer {config.LLM_API_KEY}"},
            json={
                "model": model,
                "temperature": 0.7,
                "max_tokens": getattr(config, "LLM_MAX_TOKENS", 100),
                "messages": messages,
            },
            timeout=20,
        )
    except requests.RequestException as e:
        return {"ok": False, "text": None, "error": str(e)}

    if resp.status_code != 200:
        return {"ok": False, "text": None,
                "error": f"HTTP {resp.status_code}: {resp.text[:300]}"}

    data = resp.json()
    try:
        text = data["choices"][0]["message"]["content"].strip()
    except (KeyError, IndexError, TypeError) as e:
        return {"ok": False, "text": None, "error": f"解析失败: {e}"}

    print(f"[耗时] LLM 生成 {(time.time() - t0) * 1000:.0f} ms（{len(text)} 字）")
    return {"ok": True, "text": text, "error": None}