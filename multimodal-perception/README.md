# 多模态感知交互系统（队友贡献模块）

照护机器人的**感知—决策—表达层**：摄像头姿态识别（视觉）+ 豆包语音识别（ASR）+ 火山方舟对话（LLM）+ 语音播报（TTS），装配成一条完整链路。动作意图只写入本地 `command.json`（供机械臂抓取侧读取），不做任何网络转发。

## 功能链路

执行顺序（按优先级从高到低）：**语音求助指令（任务模式） > 经期剧情问答 > 视觉痛苦姿态关怀 > 普通闲聊**。

| 模块 | 文件 | 职责 |
|------|------|------|
| 主装配 | `main.py` | 摄像头、姿态、语音、对话、播报的调度与状态机 |
| 姿态识别 | `pose_estimator.py`、`posture.py` | MediaPipe 姿态估计 + 姿态分类（捂腹/蜷缩/摔倒/蹲坐/踱步等） |
| 语音识别 | `speech.py` | VAD 端点检测 + 豆包录音文件识别大模型 2.0 |
| 对话 | `llm_chat.py`、`config.py` | 火山方舟 LLM（看护模式 / 闲聊模式双提示词隔离） |
| 语音播报 | `tts.py` | 火山柔美女友 2.0 → edge-tts 免费兜底 → pyttsx3 三级回退 |
| 生理周期 | `period_config.json` | 经期推算与剧情状态（`period_remind_state.json` 运行时生成） |

看护事件均带防抖：捂腹 ≥2.5s、摔倒 ≥4s、深夜踱步 ≥5s、长时间无动作告警（demo 30s / 正式 10h）等，参数集中在 `config.py`。

## 快速开始（Python 3.10）

```bash
pip install -r requirements.txt
python main.py
```

1. **填 API Key**（打包时已脱敏）：`config.py` 里的 `LLM_API_KEY`（[火山方舟](https://console.volcengine.com/ark)）、`ASR_API_KEY`（[豆包语音](https://console.volcengine.com/speech/new/setting/apikeys)）；`TTS_VOLCANO_API_KEY` 可选，留空则用 edge-tts 免费女声
2. 摄像头设备号在 `main.py` 顶部 `CAMERA_INDEX = 0`
3. 演示按键：**A** 模拟捂肚子触发经期剧情 / **T** 拿纸巾 / **W** 拿水杯 / **Q** 退出
4. 想重新演示经期剧情：删除 `period_remind_state.json` 后重启

## 与机械臂侧的衔接

`command.json` 中的动作意图（如 `fetch_painkiller` / `fetch_water` / `fetch_tissue`）是感知层与执行层的交接点，由 `robot-app/` 的抓取任务消费。

> ⚠️ 安全提示：`config.py` 填入密钥后请勿外传；仓库中的版本始终保持脱敏状态。
