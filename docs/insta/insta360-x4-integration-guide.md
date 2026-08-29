# Insta360 X4 接入 Panthera-HT 调试路线（可执行）

> 目标：把影石 X4 作为腕部视觉输入（eye-in-hand）接入 Panthera-HT，打通「图像采集 → 多模态识别 → 坐标变换 → 机械臂抓取」链路。
> 安装与走线的硬件需求见同目录 `insta360-x4-mount-requirements.md`。
> 环境约定：Windows 源码根目录 `F:\hackson\独居女性`，控制与脚本在 WSL Ubuntu（distro `Ubuntu-24.04`，conda 环境 `panthera`）运行，Demo 端口 5001。
>
> **Phase 0 实测记录（2026-08-29）**：
> - VID:PID = `2e1a:0003`（USB 摄像头模式，Windows 识别为 Camera 类设备 `Insta360 X4`，usbipd BUSID 1-4）；`bind-camera-to-wsl.ps1` 默认值已同步；
> - 相机当前以 UVC 形态枚举，与规划书 2.0 路线决策一致（安卓模式不可用，启用 UVC 路线）；
> - 待补：OBS 实测分辨率/视野档位（含是否存在单镜头档）。
>
> **Phase 2 故障记录（2026-08-29，usbipd attach 三连败 → 根因已定）**：
> - 现象：`usbipd attach --wsl` 每次均在总线复位瞬间把相机打挂（变为「未知 USB 设备（设备描述符请求失败）」，VID:PID 变 0000:0002），需物理重插恢复；换口、干净重绑（detach→unbind→bind→attach）均无效；
> - 证据：`DEVPKEY_Device_Parent` = `USB\ROOT_HUB30`，相机以 **SuperSpeed（USB 3.x）** 连接；
> - 根因：WSL 的 vhci 虚拟控制器只支持 USB 2.0 速率，usbipd-win 无法转发 SuperSpeed 设备，复位即掉线；初期误判为供电问题（官方须知 Q23 症状相似）；
> - 对策：让相机以 USB 2.0 速率接入（USB 2.0 Hub / USB 2.0 线 / USB2-only 口），MJPG 2880×1440@30 约 200+ Mbps，USB 2.0 的 480 Mbps 足够；
> - 操作前提：attach 前先确认保活会话（WSL2 空闲会关机）；每次开发前重跑绑定脚本；`-Detach` 归还 Windows。
>
> **Phase 2 故障记录补充（同日，USB 2.0 降速成功后遇到第二层阻断，路线终结）**：
> - 用户改用 USB 2.0 连接后，`vhci_hcd: speed(3) speed_str(high-speed)`，设备成功在 WSL 枚举，不再掉线——第一层问题确认解决；
> - 新阻断（三次 100% 稳定复现）：内核日志 `config 1 has an invalid interface number: 4 but max is 3` ×2、`config 1 has no interface number 2`——X4 UVC 固件的配置描述符本身畸形（接口编号 0/1/3/4、缺 2，Windows 容忍、Linux 拒绝）；随后 `vhci_hcd: connection closed`（描述符解析后约 0.2 秒内被 usbipd 侧关闭）→ `can't set config #1, error -110` → `/dev/video*` 不出现；
> - 已排除：停用 VirtualCameraService 后重试仍同样失败；换端口/重插无效；
> - **结论：X4 无法经 usbipd 直通 WSL（当前 usbipd-win + WSL 内核 6.18 下），属设备固件描述符缺陷与 usbip 链路的兼容性问题，软件侧无法绕过（除非重编 WSL 内核改描述符校验，代价高且不保证有效）**；
> - **替代方案（已定）：Windows 侧采集（ffmpeg/OpenCV 直读 UVC）+ MJPEG HTTP 服务（端口 8200）→ WSL 取帧**，架构与规划书 SDK 路线 P1/P2 的桥接层完全一致，`x4_sdk_bridge.py` 接口不变；或换用标准 UVC 相机直通。

## 技术路线结论（为什么是这条路）

| 候选接口 | 结论 | 依据 |
|---|---|---|
| USB-C UVC「USB 摄像头」模式 | **采用** | X4 官方支持，出流稳定，单线供电+数据，可被 OpenCV/V4L2 直接读取 |
| HDMI 采集卡 | 不可行 | X4 无 HDMI 输出 |
| Wi-Fi / Insta360 App 网络流 | 不采用 | 依赖 App，无官方开放协议，延迟与稳定性不可控 |
| Windows 捕获后转推（备选） | 仅备选 | 若 usbipd 进 WSL 不稳定，Windows 侧 OpenCV 抓帧 + MJPEG HTTP 推给 WSL |

**数据链路**：
```
X4 --USB-C--> Windows --> usbipd-win --> WSL /dev/video0
  --> OpenCV(panthera conda) --> 多模态模型服务(建议 FastAPI on Windows GPU)
  --> 像素->相机系->基座系 --> inverse_kinematics --> moveJ/moveL --> gripper
```

**抓取策略**：X4 为单目无深度相机，采用 **look-then-move**（停稳 → 拍帧 → 检测 → 平面假设求 3D → IK → 运动）。不做实时视觉伺服，规避 usbipd 与全景流的延迟问题。

---

## Phase 0 接口形态确认（Windows 侧）

目的：确认 X4 能以 UVC 摄像头形态出流，并记录设备标识。

1. X4 开机，用原装数据线（须支持数据传输）连接电脑，相机屏幕弹出 USB 模式菜单，选择 **「USB 摄像头」**。
2. 打开 Windows「相机」应用，确认识别为 `Insta360 X4` 且画面正常。
3. 记录设备 VID:PID（后续 usbipd 绑定用）：
   ```powershell
   # PowerShell
   Get-PnpDevice -Class Camera,Image | Where-Object { $_.FriendlyName -like '*Insta360*' } |
     Get-PnpDeviceProperty -KeyName 'DEVPKEY_Device_InstanceId' | Select-Object -ExpandProperty Data
   # 形如 USB\VID_2E1A&PID_xxxx\... ，记下 VID_2E1A&PID_xxxx 对应的小写 vid:pid（如 2e1a:xxxx）
   ```
4. 确认输出规格（X4 Webcam 模式下分辨率/FOV 由外部软件决定，机内设置不生效）：
   - OBS 添加「视频采集设备」，选 `Insta360 X4`，取消「使用预设」，分辨率设 2880×1440、帧率 30 → 得到全景 2:1 等距柱状画面；
   - 尝试其他分辨率档位，观察是否出现单镜头视野档位（若有，优先固定单镜头档，标定更简单）。

**判据**：能稳定出流 ≥ 1 分钟不中断 → UVC 路线成立。
**记录**：把实际 `vid:pid`、最终选定的分辨率/视野档位填入本文档顶部与 `scripts/bind-camera-to-wsl.ps1`。

## Phase 1 Windows 侧捕获验证 + 模型服务接口

目的：确认画质可用，并定义多模态模型的对接形态。

1. 在 Windows 用 ffmpeg 抓单帧验证画质与曝光：
   ```powershell
   ffmpeg -f dshow -i video="Insta360 X4" -frames:v 1 F:\hackson\独居女性\docs\insta\x4_test_frame.jpg
   ```
   （无 ffmpeg 可跳过，以 Phase 2 WSL 侧取帧为准。）
2. **多模态模型服务接口约定**（模型跑在 Windows 侧 GPU，WSL 通过 HTTP 调用）：
   - `POST http://<windows-host-ip>:8100/predict`，body：`{"image_base64": "...", "prompt": "找出可抓取物体"}`
   - 返回：`[{"label": "cup", "bbox": [x1,y1,x2,y2], "score": 0.93}]`
   - WSL 访问 Windows 宿主 IP：`wsl.exe -d Ubuntu-24.04 -- bash -lc "cat /etc/resolv.conf | grep nameserver"`
   - 备选：模型若改跑在 WSL 内，直接同进程函数调用，删除本节网络依赖。

## Phase 2 WSL 设备映射与权限（关键）

目的：让 `/dev/video0` 出现在 WSL，OpenCV 能取流。

1. 绑定相机进 WSL（管理员脚本，已提供 `scripts/bind-camera-to-wsl.ps1`）：
   ```powershell
   .\scripts\bind-camera-to-wsl.ps1 -VidPid 2e1a:xxxx   # 用 Phase 0 记录的实际值
   ```
2. WSL 侧确认设备与格式：
   ```bash
   sudo apt-get install -y v4l-utils
   ls -l /dev/video*
   v4l2-ctl -d /dev/video0 --list-formats-ext
   ```
   期望：格式列表中出现 2880×1440@30（MJPG 或 YUYV）。
3. 权限不足（Permission denied）时：
   ```bash
   sudo usermod -aG video $USER   # 之后重启 WSL：wsl --shutdown 再进
   ```
4. OpenCV 取流冒烟（在 `panthera` conda 环境）：
   ```bash
   conda activate panthera
   pip install opencv-python   # 若未装
   python - <<'EOF'
   import cv2
   cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2880)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
   ok, frame = cap.read()
   print("read ok:", ok, None if frame is None else frame.shape)
   if ok: cv2.imwrite("/tmp/x4_wsl_frame.jpg", frame)
   cap.release()
   EOF
   ```
   **判据**：打印 `read ok: True (1440, 2880, 3)`，`/tmp/x4_wsl_frame.jpg` 图像正常。

**常见问题**：
- WSL2 没有原生 USB 直通，必须经 `usbipd-win`（与机械臂串口 `map-usb-serial-to-wsl.ps1` 同一机制）。
- Windows 侧若正打开相机（相机应用 / OBS / 会议软件），WSL 看不到设备 → 先关闭占用方再 `attach`。
- `usbipd attach` 是临时绑定，WSL 重启或休眠后需重新运行脚本（`-Detach` 可解绑归还 Windows）。
- 若反复掉线 → 换直连主板后置 USB 口，避免无源 Hub。

## Phase 3 成像形态固定与内参标定

目的：得到可信的相机内参，供像素→射线反投影。

1. **固定成像形态**：选定 Phase 0 中确认的输出档位（优先单镜头视角；若只有全景档，则使用等距柱状流的某一固定裁切/投影方式），整个项目期间不再更改。把决定写进本节：
   - [ ] 已固定档位：`____`（分辨率 / 视野 / 处理方式）
2. 打印标定板：棋盘格 9×6 内角点、方格 25 mm（或 ChArUco 板），平整贴硬板上。
3. 相机固定于最终腕部支架位（未装机时可用支架固定不动），**相机不动、移动标定板**，摆 15~20 个不同距离/角度/偏转的姿态，每姿态拍 1 张清晰图（机械臂静止或纯手动）。
4. 标定脚本（单镜头鱼眼用 `cv2.fisheye`；全景档需先对等距柱状图做局部透视投影再按普通针孔标定）：
   ```python
   # 核心调用（伪代码，完整脚本实施时落到 robot-app/calibration/）
   import cv2
   K = np.zeros((3, 3)); D = np.zeros((4, 1))
   rms, K, D, _, _ = cv2.fisheye.calibrate(obj_pts, img_pts, img_size, K, D,
                                           flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC)
   # 保存 K、D、img_size、档位描述到 calib_x4.yaml
   ```
5. **判据**：`rms` 重投影误差约 ≤ 1.0 px；用 `cv2.fisheye.undistortImage` 去畸变后图中棋盘格直线目视变直、边缘无明显弯曲。
6. 产出物：`calib_x4.yaml`（含 K、D、分辨率、成像档位描述、标定日期）。

## Phase 4 手眼标定（eye-in-hand，AX=XB）

目的：求相机坐标系到末端（gripper/tool）坐标系的固定变换 `T_cam2gripper`。

1. 前置：X4 已按需求书装机完毕；标定板固定在工作台上，保证机械臂带相机在 12~20 个差异位姿时，标定板始终处于所选镜头视野内（位姿之间须有**明显姿态差异**，平移为主、兼顾旋转）。
2. 采集流程（脚本放 `robot-app/calibration/`，跑通后迁移 `panthera_python/scripts/` 供 Web Scripts 面板运行）：
   - 每个位姿：
     - `q = robot.get_current_pos()` → `T_base_tool = robot.forward_kinematics(q)['transform']`（SDK API-005）
     - 拍帧 → 检测板角点 → `cv2.solvePnP` 得 `T_target_cam`
   - 求解：
     ```python
     R_cam2g, t_cam2g = cv2.calibrateHandEye(
         R_base2g_list, t_base2g_list,      # T_base_tool 拆出
         R_target2cam_list, t_target2cam_list,
         method=cv2.CALIB_HAND_EYE_PARK)
     ```
   - 注意：`forward_kinematics` 返回的是 base→tool，需转换到 calibrateHandEye 的 gripper→base 语义（即对 `T_base_tool` 取逆），实施时逐条验证矩阵方向。
3. 保存 `handeye_x4.yaml`（`T_cam2gripper` 4×4 + 采集位姿数 + 残差）。
4. **触点验证**（判据 ≤ 5 mm）：
   - 板上选一个已知角点，由当前帧检测 + 内参反投影 + `T_cam2gripper` + FK 换算到基座系 3D 点；
   - `inverse_kinematics` 求解 → `moveJ`（vel ≤ 0.3）让夹爪尖端慢速移到该点上方 5 cm → `moveL` 缓慢下降；
   - 用尺实测尖端与角点的偏差，重复 3 个不同位置取平均。

## Phase 5 抓取闭环（look-then-move）

目的：把识别结果变成一次完整抓取。脚本草稿放 `robot-app/`，跑通后迁 `panthera_python/scripts/8_vision_grasp_x4.py`。

1. 流程：
   ```
   moveJ 到观察位（固定安全位姿）→ 停稳 → 取帧
     → POST /predict（多模态模型，返回目标像素区域）
     → 像素中心 + 内参 → 相机系射线
     → 与桌面平面求交（桌面平面在基座系中事先量测，写入配置）
     → T_cam2gripper + FK → 基座系 3D 目标点
     → 安全检查（桌面安全区包围盒内、可达性）
     → inverse_kinematics → moveJ 预抓取位（目标上方 5 cm）
     → wait_for_position 到位确认 → moveL 垂直下降
     → gripper_close → moveL 抬升 → 搬运 → gripper_open
   ```
   - 单目无深度的 3D 恢复依赖「桌面平面假设」：目标必须位于已知平面上；非平面目标需后续补充结构光/深度估计。
2. 安全约束（写死在脚本中）：
   - `moveJ`/`moveL` 速度 vel ≤ 0.5 rad/s（观察与接近段 ≤ 0.3）；
   - `max_tqu` 限力（参考知识库 HW-001 关节力矩上限的 30~50%）；
   - 目标点必须落在桌面安全区包围盒内，越界直接中止；
   - 下降前 `wait_for_position(target, tolerance)` 确认到位。
3. 调试顺序：
   - 先在 `./backend.sh --demo`（panthera_sim.py 仿真）或纯脚本 dry-run 模式验证逻辑（打印目标点不真动）；
   - 真机时一人手悬在急停/断电开关旁监护，首次只跑「移动到目标上方 5 cm 即停」。

## Phase 6 腕部安装注意事项（工程清单）

- **重量与质心**：相机+支架约 250 g，远低于 3.5 kg 负载，但改变末端质心 → 视觉任务限速 0.3~0.5，避免大加速度。
- **运动模糊**：X4 无机械快门控制、暗光快门慢 → 必须「停稳再拍」（`wait_for_position` 后再取帧，可再等 100~200 ms 消振）。
- **散热**：X4 长时间 Webcam 模式发热大、可能过温自动关机 → 实测连续 30 分钟（需求书验收项 3）；避免镜头对准强光源与热风机。
- **掉线预案**：usbipd attach 失效/相机过温重启后，脚本需检测 `cap.read()` 失败并显式报错提示重连，不得静默继续。
- **标定一致性**：相机支架一旦拆装，`T_cam2gripper` 失效，须重跑 Phase 4；支架螺栓建议点胶防松。
- **官方参考**：视觉扩展仓库 `github.com/HighTorque-Robotics/Panthera_HT_SDK_Extensions`（d405 手眼标定 + 视觉伺服示例，可对标改造）。

---

## 阶段产出物汇总

| 阶段 | 产出 | 判据 |
|---|---|---|
| Phase 0 | VID:PID、固定分辨率档位记录 | OBS 稳定出流 |
| Phase 1 | 模型服务接口约定 + 测试帧 | 画质目视可用 |
| Phase 2 | `/dev/video0` + `/tmp/x4_wsl_frame.jpg` | OpenCV 读到 (1440,2880,3) |
| Phase 3 | `calib_x4.yaml` | rms ≤ ~1.0 px，去畸变直线变直 |
| Phase 4 | `handeye_x4.yaml` | 触点验证 ≤ 5 mm |
| Phase 5 | `8_vision_grasp_x4.py` | demo 模式跑通一次完整抓取循环 |
