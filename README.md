# 独居女性照顾型机器人（Panthera-HT × Insta360 X4）

黑客松项目：用 **Panthera-HT 六自由度机械臂** + **Insta360 X4 全景相机**，搭建"看见物体 → 自动抓取"的照护机器人原型。核心命题是在消费级相机与非标准硬件约束下，用最少的工程量打通"感知—决策—执行"闭环。

## 已完成的成果

- **示教—精炼—回放闭环**：重力补偿拖动示教 → 轨迹精炼（逐路点速度前馈，消除回放锯齿顿挫）→ 回放执行（夹爪堵转检测、事件暂停）
- **数字孪生 Web 前端**：3D 实时预览、关节/笛卡尔控制、示教录制、任务管理、视觉测试面板
- **视觉引导抓取链路**：Windows 取帧服务 → HTTP 送帧 → WSL 检测判定 → 触发回放，机械臂侧零新增代码
- **视觉偏移补偿（V2，离线验证通过）**：像素映射标定 + 阻尼最小二乘数值 IK，把轨迹抓取段平移到物体实际位置；段识别、限幅、超差拒绝等安全机制齐全（真机标定未完成，见"已知问题"）
- **多模态感知交互（队友协作模块）**：MediaPipe 姿态识别 + 豆包语音识别 + 火山方舟对话 + 语音播报，识别"捂肚子/摔倒/求助语音"等事件，输出取物意图（详见 `multimodal-perception/`）
- **全流程安全设计**：默认 dry-run、触发速度限幅、关节修正限幅、动作白名单、超差拒绝执行

## 硬件与软件架构

```
[Insta360 X4] ──USB──▶ [Windows: x4_frame_server.py :8200] ──HTTP 送帧──▶ [WSL Ubuntu-24.04]
                                                                            ├─ 数字孪生后端 app.py :5000
                                                                            │   （真机控制/回放/视觉 API，serve 前端）
                                                                            ├─ 前端页面（视觉测试/示教/任务）
                                                                            └─ 视觉脚本（检测/触发/标定）
[Panthera-HT 机械臂] ◀──串口── WSL（hightorque_robot SDK，仅 Linux wheel）
```

**为什么是这个架构**（关键工程决策，详见 `docs/insta/`）：

1. X4 固件 USB 描述符畸形，`usbipd-win` 直通 WSL 不可行（已逐层验证根因），只有 Windows 能读它的 UVC 流
2. 机械臂电机 SDK 只有 Linux wheel，控制侧必须留在 WSL
3. 因此采用软件绕行：**Windows 采集 + HTTP 送帧 + WSL 控制**，相机与机械臂通过帧服务松耦合

## 目录结构

```
├── panthera-ht-sdk/                 # HighTorque 官方 SDK、数字孪生与机器人模型（MIT，原样保留）
│   ├── Panthera_digital_twin-main/  #   数字孪生：后端 app.py + 前端 + URDF
│   ├── panthera_python/             #   Python SDK、示例脚本、电机 wheel（真机控制依赖）
│   └── sdk-source/                  #   官方 SDK 源码
├── robot-app/                       # 本项目业务代码（视觉抓取）
│   ├── vision_grasp/                #   取帧服务、桥接、检测、触发、标定、轨迹精炼
│   └── data/                        #   精炼后的抓取轨迹（回放用）
├── multimodal-perception/           # 多模态感知交互系统（姿态识别+语音+LLM+TTS，API Key 已脱敏）
├── docs/                            # 项目文档：PRD、X4 集成方案、执行计划、知识库
└── scripts/                         # 启动与工具脚本（后端启动、相机绑定、SDK 检查）
```

## 快速开始

前置：Windows + WSL Ubuntu-24.04、Panthera-HT 真机、Insta360 X4（USB 接 Windows）。

```powershell
# 1. Windows 侧：启动 X4 取帧服务（8200 端口）
python robot-app\vision_grasp\x4_frame_server.py

# 2. WSL 侧：启动数字孪生后端（含真机连接，5000 端口，同时服务前端页面）
wsl -d Ubuntu-24.04 -- bash scripts/start-twin-backend.sh
```

浏览器打开 `http://localhost:5000`：连接机械臂后即可使用演示回放、示教录制、视觉测试（检测/抓取/采样标定）等功能。

重建 WSL Python 环境（如需）：`.venv-linux` 依赖见 `panthera-ht-sdk/Panthera_digital_twin-main/backend/requirements.txt`；电机控制需安装 `panthera-ht-sdk/panthera_python/motor_whl/` 下对应 Python 版本的 wheel。前端开发：`cd panthera-ht-sdk/Panthera_digital_twin-main/frontend && npm install && npm run build`。

## 安全设计

- 所有视觉触发默认 **dry-run**，显式 `--live` 才驱动真机
- 回放沿用后端既有限幅；视觉补偿额外叠加：单关节修正 ≤0.25 rad、IK 残差 ≤1 cm、偏移 >8 cm 拒绝
- 抓取段识别异常（占比过大/IK 不收敛）→ 整体拒绝执行，宁可不动
- 首次验证新轨迹一律半速

## 已知问题与未竟事项

- **视觉标定数据未收敛**：最后一轮 ±5 cm 标定采样的实际位移远小于 5 cm，像素映射未通过正交性/比例校验，偏移补偿功能停留在离线验证阶段
- **低速回放抖动**：半速回放的抖动源于低速粘滑摩擦（物理特性），全速回放平滑；已验证轨迹建议全速执行
- **X4 直通**：若 Insta360 未来修复固件描述符，可评估去掉帧服务、直接直通
- 盒子旋转角较大时夹爪方向可能与长边不匹配（未做角度校验）

## 许可证

- 本项目业务代码（`robot-app/`、`multimodal-perception/`、`scripts/`、`docs/`）：MIT
- `panthera-ht-sdk/`：HighTorque Robotics 官方 SDK 与数字孪生，遵循其自带 [MIT License](panthera-ht-sdk/LICENSE)
- Insta360 Camera SDK 为第三方版权文件，**未收录于本仓库**，请自行从 [Insta360 官网](https://www.insta360.com) 下载对应版本放入 `docs/insta/sdk/`


## 团队分工与个人贡献

HER_PROXY 是由两名成员在黑客松的一天一夜内共同完成的机器人原型，探索多模态感知、语言交互与机械臂执行在独居女性照护场景中的结合。

本仓库由团队原始仓库 Fork 而来，保留项目来源与提交历史，并补充个人负责模块的说明。

### halcyon723 — 多模态感知与交互

我主要负责项目的多模态感知与交互部分，基于 **Python、OpenCV 和 Flask** 构建数字孪生软件栈，整合 **姿态感知、语音、LLM 与 TTS**。

| 模块 | 我的工作 |
| --- | --- |
| 视觉与姿态感知 | 基于 OpenCV 处理视觉输入，提供姿态相关的感知信息 |
| 语音交互 | 集成语音输入，支持用户通过语音表达需求 |
| 大语言模型交互 | 接入 LLM，支持自然语言理解与对话回复 |
| 语音合成 | 接入 TTS，将文本回复转换为语音反馈 |
| 数字孪生与软件集成 | 基于 Python / OpenCV / Flask 整合感知和交互模块 |

我的工作重点是将视觉与语言相关能力整合到机器人原型中，支持其感知场景、接收用户表达并提供对话反馈。


### 项目阶段

本项目为黑客松期间完成的概念验证原型。仓库中的实现与现场演示体现当前完成范围；更丰富的照护场景、长期记忆与持续陪伴能力属于后续探索方向。

### 项目链接

- [团队原始仓库](https://github.com/214notfound/her-proxy)
- [halcyon723 的仓库](https://github.com/halcyon723/her-proxy)
- [项目演示视频](https://www.bilibili.com/video/BV1HX4U6LE12/)
