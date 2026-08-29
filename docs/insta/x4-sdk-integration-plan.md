# Insta360 X4 SDK 视觉接入规划书（无 SD 卡场景）

> 目标：把影石 X4 的视觉信息经电脑接入 Panthera-HT 机械臂，打通「图像采集 → 多模态识别 → 坐标变换 → 机械臂抓取」链路。
> 本规划书基于已到手并逐文件核实的 **CameraSDK 2.1.1**（头文件 + 官方示例 `example/main.cc`）编写，所有 API 名称与参数均以实收头文件为准。
> 硬件约束前提：**本机 X4 不插 SD 卡**（经与技术人员确认，无卡可实现基础功能，本文第 3 节给出该结论的完整边界）。
>
> 关联文档（均在本目录 `docs/insta/` 下）：
> - 硬件安装与走线需求：`insta360-x4-mount-requirements.md`
> - UVC Webcam 路线调试文档（本文档的备选路线）：`insta360-x4-integration-guide.md`
>
> 环境约定：Windows 源码根目录 `F:\hackson\独居女性`，机械臂控制与抓取脚本在 WSL Ubuntu（distro `Ubuntu-24.04`，conda 环境 `panthera`）运行。

---

## 1. 背景与资产盘点

### 1.1 任务目标

X4 作为腕部相机（eye-in-hand）采集画面 → 电脑接收并处理 → 输出目标坐标驱动机械臂抓取。本规划书回答两个问题：

1. **技术路线**：拿到官方 SDK 后，这条链路具体怎么实现（第 2 节）；
2. **硬件限制**：相机不插 SD 卡对实现有哪些具体限制（第 3 节）。

### 1.2 实收 SDK 资产（已解包核实，全部归档于本目录）

| 资产 | 位置（相对 `docs/insta/`） | 说明 |
|---|---|---|
| 原始赛事包 | `原始包/赛事SDK包（Windows+Linux）.rar` | 796 MB，Rar5，含下面两个平台包 |
| Windows 平台包 | `sdk/Windows_CameraSDK-2.1.1_MediaSDK-3.1.3.zip` | CameraSDK 2.1.1 + MediaSDK 3.1.3 |
| Linux 平台包 | `sdk/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip` | CameraSDK 2.1.1 + MediaSDK 3.1.1 |
| 解开即用的 Windows CameraSDK | `sdk/CameraSDK-2.1.1-win64/` | `bin/CameraSDK.dll`、`bin/CameraSDKTest.exe`（交互冒烟工具）、`include/`（camera + stream 头文件）、`lib/CameraSDK.lib`、`example/main.cc`（49KB 完整示例，本文 P0/P1 的实施底本） |

要点：

- **CameraSDK**（C++ 接口，Windows DLL / Linux .so）= 本方案主原料：设备发现、预览流、拍摄控制、状态查询。
- **MediaSDK** = 媒体文件后处理库（拼接 / 导出），以相机存储文件为输入，本次**不启用**（见 3.1 能力矩阵）。
- 两平台包的 CameraSDK 同为 2.1.1，说明 Linux 侧亦有官方库，但本方案不将其作为主路线（见 2.2）。

### 1.3 与既有文档的关系

- `insta360-x4-integration-guide.md` 的 UVC「USB 摄像头」路线**继续保留**为备选与画质兜底，其 Phase 3（内参）、Phase 4（手眼）、Phase 5（抓取闭环）流程被本文 P3/P4 复用。
- 本文档是 SDK 路线的总规划：新增内容集中在「SDK 预览流如何进电脑、如何交给 WSL 的机械臂」以及「无卡约束」。

---

## 2. 技术路线

### 2.0 路线决策记录（2026-08-29，最新）

**当前生效路线：UVC「USB 摄像头」模式**（原备选路线，现已启用）。

- **切换原因**：X4 无法进入「安卓模式」（SDK 连接形态），SDK 预览流主路线暂时不可用；经确认，UVC 模式不依赖 SD 卡、不依赖安卓模式，相机屏幕选「USB 摄像头」即可。
- **取舍说明**：放弃的是 SDK 路线的远程控制（曝光/快门/切镜头）与陀螺仪/曝光元数据；换来的是零编译成本与更高的画质上限（2880×1440 MJPG）。look-then-move 策略下，这些损失不阻断抓取闭环。
- **实施依据**：按 `insta360-x4-integration-guide.md` 的 Phase 0（接口确认）→ Phase 2（WSL 映射）执行；本文 2.1~2.6 的 SDK 方案转为**储备方案**，保留备查。
- **config 切换规则**：`camera.source` 在 WSL 侧 `/dev/video0` 验证通过后由 `mock` 改为 `v4l2`；验证通过前保持 `mock`，避免无相机时脚本报错。
- **回归条件（安卓模式修复排查）**：若后续要切回 SDK 路线，按官方须知逐项排查后再试：① 相机固件升级到官网最新版（USB 连接时模式菜单自动弹出，固件过旧可能不弹）；② 换原装数据线；③ 官方须知 Q23：安卓模式切换失败多为 USB 口电压不稳，用**带独立供电的 USB Hub** 对比验证；④ 连接时关闭所有占用 USB 的设备与软件。任一项修复后重测，成功则回到 2.1~2.6 主路线，并把结果写回本节。

### 2.1 核心结论：以 SDK 预览流为主链路，无需 SD 卡

CameraSDK 的实时预览流（`StartLiveStreaming` + `StreamDelegate::OnVideoData`）是**相机实时编码后经 USB 直传电脑的流数据，全程不落存储卡**——这正是无卡场景下可用的"基础功能"，也是本方案的取像来源。

### 2.2 SDK 能力清单（依据实收头文件核实）

| API（`ins_camera` 命名空间） | 用途 | 需要 SD 卡 |
|---|---|---|
| `DeviceDiscovery::GetAvailableDevices()` | 发现设备（USB / WiFi / 蓝牙） | 否 |
| `Camera::Open()` / `Close()` / `IsConnected()` | 会话建立与释放 | 否 |
| `SetVideoSubMode(VIDEO_LIVEVIEW)` + `SetVideoCaptureParams(..., FUNCTION_MODE_LIVE_STREAM)` | **X4 开预览流的前置步骤**（官方示例 main.cc 明确要求，X4 及以上机型必做） | 否 |
| `StartLiveStreaming(LiveStreamParam)` / `StopLiveStreaming()` | 开启 / 停止实时预览流 | 否 |
| `SetStreamDelegate(...)` → `OnVideoData`（H.264/H.265 NAL）、`OnAudioData`、`OnGyroData`、`OnExposureData` | 接收视频流与陀螺仪 / 曝光元数据 | 否 |
| `SetExposureSettings` / `SetCaptureSettings` | 曝光（ISO/快门/EV）/ 白平衡等画质控制 | 否 |
| `SetActiveSensor(SENSOR_DEVICE_FRONT/REAR/ALL)` | 镜头切换 | 否 |
| `GetStorageState()` | 存储状态查询；无卡时返回 `STOR_CS_NOCARD` | —（诊断用） |
| `GetBatteryStatus()` / `SetTemperatureHighNotification` / `SetBatteryLowNotification` | 电量与过温保护 | 否 |
| `GetPreviewParam()` | 预览参数（含 `delay_timestamp` 延迟观测字段） | 否 |
| `TakePhoto()` / `StartHDRCapture()` | 拍照，返回相机存储上的文件 URL | **是** |
| `StartRecording()` / `StopRecording()` | 录像 | **是** |
| `GetCameraFilesList()` / `DownloadCameraFile()` / `DeleteCameraFile()` | 存储卡文件列表 / 下载 / 删除 | **是** |
| `EnableInCameraStitching()` | 机内拼接（面向照片流程） | 是（照片落盘） |

官方集成须知（developer 资源中心）关键条目，与实测头文件互相印证：

- X4 预览流分辨率**固定、不可调整**（以实机出流为准，见 P0）；
- 预览流支持经 USB 传到电脑；**录制视频则必须存存储卡**；
- SDK **不能控制 Webcam（UVC）模式**——两条路线互斥，同一时刻相机只能处于其中一种 USB 形态；
- X4 经 USB 连电脑会自动弹出 USB 模式菜单（官方称 SDK 连接形态为「安卓模式」，与 UVC 路线选择的「USB 摄像头」不同项）。

### 2.3 总体架构（主路线：Windows C++ 桥接）

```
X4 --USB(安卓模式)--> Windows：x4-bridge（C++，链接 CameraSDK.dll）
    DeviceDiscovery → Open → SetVideoSubMode(VIDEO_LIVEVIEW)
    → SetVideoCaptureParams(FUNCTION_MODE_LIVE_STREAM)
    → StartLiveStreaming → StreamDelegate::OnVideoData(H.264)
    → FFmpeg 解码 → 最新帧缓冲 → MJPEG HTTP 服务（端口 8200）
    → WSL Python：robot-app/vision_grasp/x4_sdk_bridge.py 的 capture_frame()
    → 现有 vision_grasp 流水线（多模态感知 → 像素→基座系 → IK → moveJ/moveL → 夹爪）
```

架构要点：

- **桥接进程放 Windows**：SDK 是 C++ 库，Windows 侧有现成 `CameraSDKTest.exe` 可直接冒烟；WSL 的 USB 通道已被机械臂串口的 `usbipd` 映射占用，不宜再塞相机设备。
- **WSL 只走 HTTP 取帧**：与既有约定一致（模型服务 8100、Demo 5001，本服务用 **8200**）；WSL 访问 Windows 宿主 IP 用 `cat /etc/resolv.conf | grep nameserver`。
- **抓取策略维持 look-then-move**（停稳 → 拍帧 → 检测 → 平面假设求 3D → IK → 运动），不做实时视觉伺服，理由见 3.2 第 4 条。

### 2.4 路线选型对比

| 路线 | 结论 | 理由 |
|---|---|---|
| SDK 预览流 + Windows C++ 桥接（HTTP 送帧） | **主路线** | 可程序化控制曝光/快门（抑制运动模糊）、附带陀螺仪与曝光元数据、可查询状态做过温保护；相机 USB 形态由 SDK 独占管理，清晰可控 |
| UVC Webcam（usbipd 进 WSL，OpenCV 直读） | **备选/兜底** | 零编译成本、2880×1440 MJPG 画质上限更高；但无远程控制与元数据，且与 SDK 模式互斥（相机一次只能选一种 USB 形态） |
| WSL 直连 Linux CameraSDK（usbipd 相机进 WSL + pybind11 绑定） | 不推荐 | WSL USB 已被机械臂串口占用、设备冲突风险高；C++ → Python 绑定开发与维护成本高；调试不如 Windows 侧直接 |

### 2.5 分阶段实施步骤

#### P0 连接形态确认 + 官方工具冒烟（Windows，0.5~1 天）

1. X4 用原装数据线（须支持数据传输）连电脑；屏幕弹出 USB 模式菜单时选择**「安卓模式」**（SDK 连接形态；区别于 UVC 路线的「USB 摄像头」）。若菜单不弹出，先将相机固件升级到官网最新版（官方须知 Q22：USB 连接时菜单自动弹出）。
2. 运行 `docs/insta/sdk/CameraSDK-2.1.1-win64/bin/CameraSDKTest.exe`：确认设备列表打印出本机 X4（序列号 / 型号 / 固件版本）。
3. 在工具菜单中选「10: start preview live streaming」，让示例把预览流写成 `01_*.h264` / `02_*.h264` 文件；用 `ffprobe` 查看实际分辨率 / 帧率 / 码率，并用 FFmpeg 解出一帧截图目检画质。
4. 记录实测结果并回填本文档 2.6 表格；同时确认双流 `stream_index`（0/1）的含义（主/副流或左右镜头，示例仅原样落盘未注释）。

**判据**：`GetAvailableDevices` 发现设备；预览流稳定出流 ≥ 1 分钟不中断；`ffprobe` 输出记录在案。

#### P1 C++ 桥接程序 `x4-bridge`（Windows，2~4 天）

基于 `example/main.cc` 精简改写为常驻服务，目录新建 `robot-app/x4_sdk_bridge/`（规划阶段只列职责，实施时再落代码）：

- **取流**：设备发现 → `Open()` → `SetStreamDelegate` → X4 前置流程（`SetVideoSubMode(VIDEO_LIVEVIEW)` + `SetVideoCaptureParams`）→ `StartLiveStreaming(LiveStreamParam)`。`LiveStreamParam` 建议：`enable_audio = false`、`enable_gyro = true`、`using_lrv = false`（LRV 为 1440×720 低码率代理流，示例注释：开 `using_lrv` 时取到的分辨率为 1024×512，仅作监控备用）、`video_bitrate` 取示例默认 10 Mbps 量级起步。
- **解码**：`OnVideoData` 收到 H.264 NAL → FFmpeg（libavcodec）解码为 BGR；帧缓冲**只保留最新帧**（取帧语义是"停稳后拍当前"，丢弃过期帧）。
- **保护**：`SetTemperatureHighNotification` 过温 → 停流并置服务状态为不可用；启动时 `GetStorageState()`，无卡（`STOR_CS_NOCARD`）属预期，打日志继续；`GetBatteryStatus` 低电告警。
- **服务**：HTTP 端口 8200：
  - `GET /frame.jpg` → 最新帧 JPEG；缓冲为空时返回 503，禁止返回旧帧冒充新帧；
  - `GET /status` → JSON：连接 / 温度告警 / 电量 / 分辨率 / 帧率 / 累计帧数。
- **画质控制（可选增强）**：`SetExposureSettings` 固定较快快门，抑制运动模糊，配合"停稳再拍"。

**判据**：浏览器或 `curl` 连续取帧 300 秒无中断；拔线 / 相机关机时 `/status` 与取帧行为给出明确错误而非挂死。

#### P2 Python 侧接入（WSL，0.5 天）

- 新建 `robot-app/vision_grasp/x4_sdk_bridge.py`，实现模块级 `capture_frame() -> np.ndarray (BGR)`：HTTP GET `http://<windows-host-ip>:8200/frame.jpg`（IP 取 `/etc/resolv.conf` 的 nameserver），超时 3 秒；失败显式抛 `CameraError` 并附重连提示（与 `x4_camera.py` 的 VIS-005 约定一致）。可选实现 `open()`/`close()`（先 GET `/status` 探活并打印）。
- `config.yaml` 的 `camera.source` 由 `mock` 改为 `x4_sdk`。
- **现有 `X4SdkCamera` 零改动**：`x4_camera.py` 已预留「发现 `x4_sdk_bridge` 模块即委托」的桥接机制。

**判据**：`python -c "import x4_sdk_bridge; print(x4_sdk_bridge.capture_frame().shape)"` 输出真实帧尺寸；`run_grasp.py --mode dry-run` 用真帧跑通数值链路。

#### P3 标定适配（沿用集成文档 Phase 3/4）

- 单目内参 + 手眼（AX=XB）流程与集成文档一致，但**成像源换成 SDK 预览流后必须重新标定**：预览流的分辨率与畸变特性与 UVC 的 2880×1440 档不同，`calib_x4.yaml` 不得沿用。
- 成像形态二选一并固定（整个项目期间不再更改，决定写回本节）：
  - 全景拼接档（2:1 等距柱状）：视野大，边缘拉伸畸变大，标定时需先做局部透视投影；
  - 单镜头档（`SetActiveSensor` 选 FRONT，Lens A 朝作业方向）：视场小但几何简单，**优先**（与安装需求书 Lens A 朝向一致）。
- 压缩流画质可能降低角点提取精度，判据与应对措施见 3.2 第 2 条（涉及放宽判据的决定须先经确认再执行并记录）。

**判据**：重投影误差达到既定判据（默认 ≤ 1.0 px，如需调整见 3.2）；手眼触点验证 ≤ 5 mm。

#### P4 闭环联调（复用现有三模式）

- `run_grasp.py` 的 dry-run → 几何模式 → 策略模式顺序不变；`camera.source = x4_sdk` 后，mock 相机仅保留为无相机时的逻辑验证手段。
- 真机首跑只执行「移动到目标上方 5 cm 即停」，一人在急停旁监护（集成文档 Phase 5 既定安全规则）。

### 2.6 阶段产出物汇总

| 阶段 | 产出 | 判据 |
|---|---|---|
| P0 | 实测预览流规格记录（回填下表）、`CameraSDKTest` 冒烟通过 | 出流 ≥ 1 分钟稳定 |
| P1 | `robot-app/x4_sdk_bridge/`（x4-bridge 工程 + 构建脚本） | 连续取帧 300 s 无中断，异常状态可观测 |
| P2 | `robot-app/vision_grasp/x4_sdk_bridge.py` + `config.yaml` 切换 | 真帧进 dry-run 数值链路 |
| P3 | `calibration/calib_x4.yaml`、`handeye_x4.yaml`（按 SDK 流重标） | 内参/手眼判据达标 |
| P4 | 一次完整抓取闭环记录 | 演示模式跑通抓取循环 |

**P0 实测记录表（待填）**：

| 项 | 实测值 |
|---|---|
| 预览流分辨率 / 帧率 | ____ |
| 实际码率 | ____ |
| 双流 stream_index 含义 | ____ |
| 端到端延迟（`GetPreviewParam` + 秒表法） | ____ |
| 固定成像形态（全景 / 单镜头） | ____ |

---

## 3. 硬件限制分析（无 SD 卡）

### 3.1 能力矩阵

| 功能 | 无卡可用性 | 依据 |
|---|---|---|
| 实时预览流（本方案取像来源） | 可用 | 流数据实时经 USB 传出，不落卡；官方须知：「支持传输预览流」 |
| 陀螺仪 / 曝光元数据 | 可用 | `OnGyroData` / `OnExposureData` 随流回传 |
| 曝光 / 白平衡 / 镜头切换 / 状态查询 | 可用 | 均为相机在线控制指令 |
| 高清拍照（72MP 级静帧） | **不可用** | `TakePhoto` 返回的是存储卡文件 URL，无卡无落盘位置 |
| 高清录像与回传 | **不可用** | 官方须知：「录制视频则需存储在相机的存储卡中」 |
| 文件列表 / 下载 / 删除 | **不可用** | 依赖卡内文件系统；`GetStorageState` 返回 `STOR_CS_NOCARD` |
| 照片机内拼接 / MediaSDK 后处理 | **不可用** | 输入均为卡内媒体文件 |

结论与技术人员说法一致：**"边看边用"的实时视觉完全可行；"拍下来再拿出来"的高清素材不可行。**

### 3.2 对技术实现的具体限制

1. **分辨率上限被预览流锁死**：拿不到 8K/5.7K 级静帧；预览流分辨率官方声明固定不可调，实际档位待 P0 实测（候选如 3840×1920@30 / 1920×960@30 / 1440×720@30），**可能低于** UVC 路线的 2880×1440。影响：小目标在画面中的像素占比受限，检测与定位精度上限随之受限。
2. **画质为压缩流**：H.264、码率约 10 Mbps 量级（示例默认值），运动与暗光下有块状伪影。影响：小物体检测置信度、标定角点亚像素提取精度受损——内参标定的重投影误差可能达不到 ≤ 1.0 px 的既定判据。**若实测确需放宽判据、或改换更大标定板 / 更疏角点 / ChArUco 板，须先向项目负责人说明规则与影响范围、确认后再实施，并把最终决定写回本文档 2.6 与 `calib_x4.yaml` 头部注释。**
3. **无法"先录高清后处理"**：数据采集（`collect_grasp_data.py`）只能存预览流帧，训练数据画质上限就此锁定。当前策略网络输入仅 128×128（`config.yaml` train.image_size），影响可接受；未来若上高精度检测任务需重新评估（补卡即可解锁，见 3.3）。
4. **延迟**：预览流经机内编码 → USB 传输 → 解码，端到端延迟预计数十至上百毫秒，可用 `GetPreviewParam().delay_timestamp` 观测并在 P0 用秒表法实测。因此**维持 look-then-move**（停稳→拍帧→再动），禁止实时视觉伺服；取帧接口只返回最新帧、过期即弃。
5. **发热与供电**：持续预览流使机身发热明显，存在过温停机风险。措施：桥接进程必须接 `SetTemperatureHighNotification` 做过温中止与提示；连续 30 分钟出流测试沿用安装需求书验收项 3；供电验收沿用安装需求书第 5 节。

### 3.3 缓解与升级路径

- **短期（无卡即可启动）**：look-then-move + 停稳取帧对画质要求可控，预览流足以支撑抓取闭环全链路。
- **中期（补一张 SD 卡即解锁，架构不变）**：高速 UHS-I/V30 卡一张即可解锁高清拍照与录像回传；届时增加"停稳后高清单帧"增强取帧模式（`TakePhoto` + `DownloadCameraFile`），作为抓取前精细检测的可选项，在桥接程序中加一条可选分支即可，不改动现有架构。
- **兜底（现已启用，见 2.0 决策记录）**：UVC Webcam 路线（`config.yaml` 的 `camera.source: v4l2` 分支与集成文档完好）；因相机无法进入安卓模式，该路线已于 2026-08-29 升为当前生效路线，本文档其余阶段（P3/P4 流程）不变。

---

## 4. 风险清单与前置待确认项

| # | 风险 / 待确认项 | 影响 | 应对 |
|---|---|---|---|
| 1 | 预览流实际分辨率 / 帧率未知（官方称固定） | 画质与检测精度上限 | P0 实测，回填 2.6 后再定标定板参数 |
| 2 | USB 模式菜单选错（选成「USB 摄像头」） | `GetAvailableDevices` 找不到设备 | 明确选「安卓模式」；固件先升级 |
| 3 | Windows 侧相机被其他程序占用 | `Open()` 失败 | 桥接启动前关闭相机应用 / OBS / 会议软件；`/status` 暴露连接状态 |
| 4 | 相机过温自动关机 | 取流中断 | 过温回调 + 30 分钟连续出流验收 + 掉线显式报错（不静默继续） |
| 5 | 压缩伪影影响标定精度 | 内参 / 手眼残差超标 | 判据是否放宽须先确认后执行（见 3.2 第 2 条） |
| 6 | WSL → Windows HTTP 跨网连通 | 取帧失败 | 按 `resolv.conf` nameserver 约定；Windows 防火墙放行 8200 |
| 7 | 双流 `stream_index` 语义未注释 | 取错流 | P0 用 `ffprobe` + 画面比对确认 |

**透明化规则**：凡影响全局的规则性决定（成像档位固定、码率设定、标定判据调整、数据采集口径），一律先说明、确认后执行，并同步写回本文档 2.6 与相应配置文件，不允许只存在于代码里。

---

## 5. 官方参考资料

- Insta360 SDK 官网（功能支持列表 / 申请流程）：https://www.insta360.com/sdk/home
- developer 集成须知（SDK 与 OSC 差异、预览流 / 存储卡 / Webcam 模式问答）：https://onlinemanual.insta360.com/developer/zh-cn/resource/integration
- X4 Webcam 模式说明（UVC 备选路线依据）：https://onlinemanual.insta360.com/x4/zh-cn/faq/functionality/webcam
- 本目录 `sdk/CameraSDK-2.1.1-win64/example/main.cc`：官方完整示例（设备发现、预览流、拍照下载、状态查询），P1 实施底本
