# 独居女性照顾型机器人（Panthera-HT × Insta360 X4）

黑客松项目：用 **Panthera-HT 六自由度机械臂** + **Insta360 X4 全景相机**，搭建"看见物体 → 自动抓取"的照护机器人原型。核心命题是在消费级相机与非标准硬件约束下，用最少的工程量打通"感知—决策—执行"闭环。

## 已完成的成果

- **示教—精炼—回放闭环**：重力补偿拖动示教 → 轨迹精炼（逐路点速度前馈，消除回放锯齿顿挫）→ 回放执行（夹爪堵转检测、事件暂停）
- **数字孪生 Web 前端**：3D 实时预览、关节/笛卡尔控制、示教录制、任务管理、视觉测试面板
- **视觉引导抓取链路**：Windows 取帧服务 → HTTP 送帧 → WSL 检测判定 → 触发回放，机械臂侧零新增代码
- **视觉偏移补偿（V2，离线验证通过）**：像素映射标定 + 阻尼最小二乘数值 IK，把轨迹抓取段平移到物体实际位置；段识别、限幅、超差拒绝等安全机制齐全（真机标定未完成，见"已知问题"）
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
├── robot-app/                       # 本项目业务代码
│   ├── vision_grasp/                #   视觉抓取：取帧服务、桥接、检测、触发、标定、轨迹精炼
│   └── data/                        #   精炼后的抓取轨迹（回放用）
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

- 本项目业务代码（`robot-app/`、`scripts/`、`docs/`）：MIT
- `panthera-ht-sdk/`：HighTorque Robotics 官方 SDK 与数字孪生，遵循其自带 [MIT License](panthera-ht-sdk/LICENSE)
- Insta360 Camera SDK 为第三方版权文件，**未收录于本仓库**，请自行从 [Insta360 官网](https://www.insta360.com) 下载对应版本放入 `docs/insta/sdk/`
