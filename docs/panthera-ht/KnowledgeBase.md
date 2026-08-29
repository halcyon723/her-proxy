# Panthera-HT 知识库（数据库）

> 本文件是 Panthera-HT 机械臂项目的结构化知识库，按主题分区，每个条目带唯一 ID，供后续检索问答使用。
> 信息来源：本地 `panthera-ht-sdk` 官方仓库 + GitHub HighTorque-Robotics 组织官方仓库。

---

## 1. 硬件参数

### [HW-001] 机械臂基本规格（Panthera-HT）
- 类型：6 轴开源机械臂，使用高擎机电（HighTorque）行星关节模组
- 质量：4.35 kg；臂展：860 mm；折叠尺寸：460 mm
- 最大负载：3.5 kg；最大关节扭矩（峰值）：36 Nm
- 供电：建议 24V 15A 可调电源（官方套装附 220V 转 24V 15A 适配器）
- 电机配置：6 关节 + 1 夹爪（上位机显示 7 个电机）
- 关节力矩上限（配置文件）：`[21.0, 36.0, 36.0, 21.0, 10.0, 10.0]` Nm
- 关节限位（Follower.yaml，rad）：lower `[-2.4, -0.1, -0.1, -1.6, -1.7, -2.5]`，upper `[2.4, 3.2, 4.0, 1.6, 1.7, 2.5]`
- 夹爪行程（rad）：0.0 ~ 2.0（`gripper_open` 默认 pos=1.6，`gripper_close` 默认 pos=0.0）
- 速度限制：1.0 rad/s（各关节）；加速度限制：2.0 rad/s²
- Mini 型号 Panthera-HT_S：3 kg / 641 mm 臂展 / 2.85 kg 负载 / 21 Nm，夹爪尺寸相同

### [HW-002] 通信与接线
- 电机通过通信板连接，正常连接可见 7 个 `/dev/ttyACM*` 设备
- 串口权限：`sudo chmod -R 777 /dev/ttyACM*`；可用 udev 规则 `KERNEL=="ttyACM*", MODE="0777"` 自动赋权
- 主从遥操接线：从臂接 CAN 口 1，主臂接 CAN 口 2
- 底层支持最多 7 路 CAN 总线，每条总线最多 30 个电机
- 电机电源按钮亮绿灯 = 已上电；掉电时须扶好机械臂防止跌落

---

## 2. 软件生态与仓库地图

### [SW-001] 官方仓库列表（HighTorque-Robotics）
| 仓库 | 用途 |
|---|---|
| `Panthera-HT_Main` | 主项目：介绍、参数表、能力状态、资料库入口 |
| `Panthera-HT_SDK` | C++/Python SDK：电机控制、运动学、动力学、轨迹规划 |
| `Panthera-HT_Host` | Web 上位机（本项目中为 `Panthera_digital_twin-main/` 数字孪生前后端） |
| `Panthera_HT_SDK_Extensions` | **视觉扩展仓库**：手眼标定、视觉伺服、d405 相机示例 |
| `Panthera-HT_ROS2` | ROS2 驱动/控制/仿真（MoveIt、笛卡尔阻抗） |
| `Panthera-HT_lerobot` | LeRobot 模仿学习数据采集与推理 |
| `Panthera-HT_Model` | SolidWorks 设计文件、钣金图、3D 打印文件、BOM |
| `Panthera-HT_RoboTwin` | RoboTwin 2.0 仿真单双臂采集 |
| `Panthera_digital_twin` | 数字孪生可视化项目（数字孪生前后端源头） |

### [SW-002] 官方已有视觉能力（可直接对标）
- 视觉伺服（Visual Servoing）
- 色块视觉跟踪
- GraspNet 抓取位姿估计 Demo
- d405 相机手眼标定（Extensions 仓库）
- 视频参考：GraspNet `BV13KcDzLE3F`；色块跟踪 `BV1JQPfz4EPN`；SDK 配置教程 `BV1SxwYzhEai`
- 交流群：QQ 1035440629

### [SW-003] 本地项目目录映射（本工作区）
- `panthera-ht-sdk/` = 官方 Host + 数字孪生 + SDK 脚本（对应 Panthera-HT_Host 内容）
- `panthera-ht-sdk/sdk-source/` = 官方 `Panthera-HT_SDK` 源码（C++ + Python）
- `panthera-ht-sdk/Panthera_digital_twin-main/` = 数字孪生前后端（Demo 即此）
- `panthera-ht-sdk/panthera_python/` = Host 侧 SDK（脚本被 Web Scripts 面板读取）
- `robot-app/` = 本项目业务代码；`docs/` = 文档；`scripts/` = WSL 映射/启动脚本
- 运行方式：Windows 源码 → 映射到 WSL Ubuntu 运行；Demo 服务 `http://localhost:5001`
- WSL 中 conda 环境名 `panthera`；`.venv-linux` 为 Linux 虚拟环境

---

## 3. Python SDK（Panthera 类）API

### [API-001] 初始化与配置
- `Panthera(config_path=None)`：默认加载 `Leader.yaml`；真机控制用 `robot_param/Follower.yaml`
- 配置文件（yaml）包含：关节限位、夹爪限位、max_torque、速度/加速度限制、URDF 路径（`base_link`/`tool_link`）、控制默认参数、笛卡尔参数
- 笛卡尔参数（`moveit_cartesian`）：`eef_step=0.002`（末端步长 m）、`jump_threshold=1.5`（关节跳变阈值 rad）、`resample_dt=0.01`（100Hz 重采样）
- 运动学基于 `pin`（pinocchio 的 pip 包，注意不是测试框架 `pinocchio`）

### [API-002] 状态读取
- `get_current_state()` 全部状态；`get_current_pos()` 关节角 [6,]；`get_current_vel()`；`get_current_torque()`
- 夹爪：`get_current_state_gripper()` / `get_current_pos_gripper()` / `get_current_vel_gripper()` / `get_current_torque_gripper()`

### [API-003] 关节空间控制
- `Joint_Pos_Vel(pos, vel, max_tqu=None, iswait=False, tolerance=0.1, timeout=15.0)`：单关节位置+速度+最大力矩控制
- `Joint_Vel(vel)`：速度控制
- `moveJ(pos, duration, max_tqu=None, iswait=False, tolerance=0.1, timeout=15.0)`：所有关节同步到达（五次/七次多项式规划）
- `pos_vel_tqe_kp_kd(pos, vel, tqe, kp, kd)`：五参数 MIT 模式（位置+速度+前馈力矩+Kp+Kd）
- 到位检测：`check_position_reached(target, tolerance)`、`wait_for_position(target, tolerance, timeout)`
- 底层：`motor_send_cmd()`、`send_get_motor_state_cmd()`、`set_stop()`、`set_reset()`、`set_timeout(ms)`

### [API-004] 夹爪控制
- `gripper_control(pos, vel, max_tqu=0.5)`：位置速度控制
- `gripper_control_MIT(pos, vel, tqe, kp, kd)`：MIT 控制
- `gripper_open(pos=1.6, vel=0.5, max_tqu=0.5)`、`gripper_close(pos=0.0, vel=0.5, max_tqu=0.5)`

### [API-005] 运动学
- `forward_kinematics(joint_angles=None)` → `{'position':[x,y,z], 'rotation':R, 'transform':T, 'joint_angles':q}`
- `inverse_kinematics(target_position, target_rotation=None, init_q=None, max_iter=1000, eps=1e-4)` → 关节角 [6,] 或 None（DLS 阻尼最小二乘，支持多初始值）
- `get_jacobian(joint_angles=None)`：雅可比矩阵（视觉伺服速度控制核心）
- `get_manipulability()`：可操作度；`compute_damped_pseudoinverse(J, damping=0.01)`

### [API-006] 笛卡尔控制（视觉抓取核心）
- `moveL(target_position, target_rotation=None, duration=None, use_spline=True, max_tqu=None)` → bool
  - 流程：`compute_cartesian_path`（eef_step 插值+SLERP 姿态插值+跳变检测）→ `compute_time_parameterization` → 可选样条平滑 → `Joint_Pos_Vel` 执行
- `compute_cartesian_path(waypoints, avoid_collisions=False)` → `(joint_trajectory, fraction)`，fraction<0.99 视为不完整
- `compute_time_parameterization(joint_trajectory, duration=None)` → timestamps
- `smooth_trajectory_spline(joint_trajectory, timestamps)` → `(轨迹, 时间戳, 速度)`
- `_execute_trajectory(joint_trajectory, timestamps, velocities, max_tqu=None)` → bool
- `rotation_matrix_from_euler(roll, pitch, yaw)`

### [API-007] 动力学
- `get_Gravity(q)` 重力矩；`get_Coriolis(q,v)` / `get_Coriolis_vector(q,v)`；`get_Mass_Matrix(q)`；`get_Inertia_Terms(q,a)`
- `get_Dynamics(q,v,a)`：τ = M(q)a + C(q,v)v + G(q)
- `get_friction_compensation(vel, Fc, Fv, vel_threshold=0.01)`：τ_f = Fc·sign(v) + Fv·v

### [API-008] 轨迹插值
- `quintic_interpolation(start, end, duration, t)`：五次多项式（速度/加速度连续）
- `septic_interpolation(start, end, duration, t)`：七次多项式（加加速度连续）
- `septic_interpolation_with_velocity(start, end, start_vel, end_vel, duration, t)`

---

## 4. SDK 示例脚本索引（scripts/，编号即学习路线）

### [SCR-001] 脚本清单
| 脚本 | 功能 |
|---|---|
| `0_robot_get_state.py` | 状态读取（首次验证必跑） |
| `0_robot_set_zero.py` | 设置零位 |
| `1_Joint_PosVel_control.py` | 位置速度控制 |
| `1_Joint_Vel_control.py` / `1_Joint_PD_control.py` | 速度/PD 控制 |
| `1_moveJ_control.py` | moveJ 同步运动 |
| `1_forward_kinematics_test.py` / `1_inverse_kinematics_test.py` | FK/IK 测试 |
| `2_inv_PosVel_control.py` | 基于 IK 的位置控制 |
| `2_gravity_compensation_control.py` | 重力补偿（拖动示教） |
| `2_gravity_friction_compensation_control.py` | 重力+摩擦补偿 |
| `2_Jointimpendence_control_with_gra_pd.py` / `_gra_fri_pd.py` | 关节阻抗控制 |
| `3_interpolation_control_zeroVel.py` / `_nozeroVel.py` | 插值轨迹控制 |
| `3_sin_trajectory_control.py` | 正弦轨迹 |
| `4_impedance_trajectory_control_with_gra_pd.py` | 轨迹阻抗控制 |
| `5_teleop_control.py` | 主从遥操 |
| `5_record_trajectory.py` / `5_replay_trajectory.py` | 轨迹记录/回放（.jsonl） |
| `6_moveL_pos_control.py` / `6_moveL_rotate_control.py` | 笛卡尔直线/旋转运动 |
| `7_keyboard_cartesian_pos_control.py` | 键盘笛卡尔位置（IK） |
| `7_keyboard_cartesian_vel_control.py` | 键盘笛卡尔速度（雅可比） |
| `7_gamepad_cartesian_vel_control.py` | 手柄笛卡尔速度控制 |
| `7_vr_cartesian_control.py` / `7_vr_dual_cartesian_control.py` | VR 笛卡尔控制 |
| `7_gello_teleop.py` | GELLO 遥操 |
| `motor_example/01~09` | 底层电机控制（位置/速度/力矩/电压/电流/零位等） |

### [SCR-002] 视觉抓取最相关脚本
- `6_moveL_pos_control.py`：moveL 直线运动示范（抓取接近段用）
- `7_keyboard_cartesian_vel_control.py`：雅可比速度控制范式（视觉伺服闭环用）
- `1_inverse_kinematics_test.py`：IK 验证
- 轨迹文件格式：`.jsonl`，每帧含关节角/时间戳/`config_path`（改 `Follower.yaml` 可切换主从臂回放）

---

## 5. 数字孪生后端（Host/Web）

### [HOST-001] 启动方式
- 一键安装：`./install_oneclick.sh`（conda 环境 `panthera` + SDK + 数字孪生依赖）；仅数字孪生：`./install.sh`
- 真机：`./backend.sh`（= `--live --config .../Follower.yaml --port 5000`）
- Demo：`./backend.sh --demo`（panthera_sim.py 仿真，不动真机）
- 前端开发服务器：`./frontend.sh`（默认 3000，热更新）；正式页面由 5000 端口托管 `npm run build` 产物
- 本项目 Demo 端口为 5001（见根目录 README）

### [HOST-002] 控制模式（CONTROL MODE）
- `position`：位置控制（Joints 面板/Send Position 在此模式用）
- `gravity_comp`：重力补偿（浮动拖动示教，夹爪释放）
- `gravity_friction`：重力+摩擦补偿
- `impedance`：阻抗控制（柔顺，夹爪轻量 MIT 保持）
- `trajectory`：轨迹执行时的内部临时模式
- 从补偿/阻抗切回 position 时有平滑回零策略，避免跳变

### [HOST-003] REST API 端点（app.py，Flask）
- `GET /api/config`：控制频率、连接状态、当前模式、末端 link、末端偏移
- `GET /api/status`：实时状态（WebSocket/轮询，含关节位置、FK、模式、夹爪）
- `POST /api/move_joint`：单关节运动；`POST /api/move`：多关节运动；`POST /api/home`：回零；`POST /api/stop`：停止
- `POST /api/set_mode`：切换控制模式（'position'/'gravity_comp'/'gravity_friction'/'impedance'）
- `GET /api/scripts` / `GET /api/scripts/output` / `GET /api/scripts/log` / `POST /api/scripts/run` / `POST /api/scripts/stop`：运行 `panthera_python/scripts/` 下一层 `.py` 脚本
- 静态资源：`/arm_description/<file>`、`/Panthera-HT_description/<file>`
- 关键文件：后端入口 `backend/app.py`；仿真 `backend/panthera_sim.py`；前端 `frontend/src/main.js` 和 `frontend/src/ui/`

---

## 6. 环境安装与故障排除

### [ENV-001] Python SDK 安装
- 系统：Ubuntu 20/22/24 + Miniconda；conda 环境：`conda create -n panthera python=3.10`
- 方式1（Ubuntu 22 推荐）：`pip install motor_whl/hightorque_robot-1.2.0-cp310-cp310-linux_x86_64.whl`
- 方式2（源码编译）：依赖 `cmake / liblcm-dev / libyaml-cpp-dev / libserialport-dev` + `pip install pybind11`，先编译 `panthera_cpp/motor_cpp` 再编译 `panthera_python`
- 高层库：`pip install -r requirements.txt`（pyyaml>=6.0、pin>=2.6.0、scipy>=1.9.0）
- 验证：`python -c "import hightorque_robot"`；先跑 `0_robot_get_state.py`

### [ENV-002] 常见问题
- `libserialport.so.0` 缺失 → `sudo apt-get install libserialport-dev`
- `libyaml-cpp.so.0.6` 缺失 → 卸载 0.7，源码编译安装 yaml-cpp 0.6.1（`-DBUILD_SHARED_LIBS=ON`）后 `sudo ldconfig`
- URDF 加载失败 → 检查配置文件中 URDF 相对路径（相对配置文件目录）
- IK 不收敛 → 目标超出工作空间，或调整 `max_iter`/`eps`
- 电机无响应 → 检查板子开关/电源绿灯/连接线
- npm warning → `npm audit fix --force`

---

## 7. 视觉抓取扩展路线（本项目目标）

### [VIS-001] 官方参考资料
- 仓库：`github.com/HighTorque-Robotics/Panthera_HT_SDK_Extensions`（手眼标定 + 视觉伺服 + d405 相机）
- 主仓库能力表已列：视觉伺服、色块跟踪、GraspNet 位姿估计
- GraspNet 视频：bilibili BV13KcDzLE3F；色块跟踪：BV1JQPfz4EPN

### [VIS-002] 视觉识别抓取标准流程（eye-to-hand 外部固定相机）
1. **相机安装与内参标定**：eye-in-hand（装末端）或 eye-to-hand（外部固定，推荐 USB 相机/RealSense d405）
2. **手眼标定**：求解相机坐标系到基座坐标系的变换（eye-to-hand 求 T_base_cam；eye-in-hand 求 T_tool_cam，AX=XB）
3. **目标检测**：颜色/ArUco 标签（入门）→ YOLO/GraspNet（进阶）
4. **坐标变换**：像素 → 相机系（内参+深度）→ 基座系（手眼矩阵）
5. **抓取规划**：`inverse_kinematics()` 求关节角 → `moveJ()` 到达预抓取位 → `moveL()` 垂直下降
6. **执行**：`gripper_close()` 夹紧 → `moveL()` 抬升 → 搬运 → `gripper_open()` 释放
7. **伺服闭环（可选）**：`get_jacobian()` 逆雅可比把图像误差转成末端速度，`Joint_Vel()` 闭环
- 注：本项目采用 eye-in-hand 腕部安装（见 [VIS-004]），差异在于第 2 步求 `T_cam2gripper`（AX=XB）、第 4 步基座系坐标需叠加当前位姿的 FK，且单目无深度相机用桌面平面假设恢复 3D 点，抓取采用 look-then-move 而非实时伺服。

### [VIS-003] 集成到现有系统的落点
- 视觉脚本放入 `panthera_python/scripts/`（或 `sdk-source/panthera_python/scripts/`），可通过 Web Scripts 面板运行
- 视觉处理依赖（opencv、ultralytics 等）装入 WSL 的 `panthera` conda 环境
- 相机接在 Windows/WSL：USB 相机需确认 `/dev/video*` 在 WSL 可见（usbipd-win）
- 抓取安全：限速（vel≤0.5）、限力矩（max_tqu）、目标点限定在桌面安全区、下降前检测到位

### [VIS-004] Insta360 X4 腕部相机接入路线（本项目选定方案）
- 连接：仅 USB-C UVC「USB 摄像头」模式可行（X4 无 HDMI 输出；Wi-Fi 取流依赖 Insta360 App 不可靠）；Webcam 模式下分辨率/FOV 由外部软件决定、机内设置不生效，全景档输出 2880×1440@30 等距柱状流，项目启动时须固定一种成像形态不再更改（优先单镜头档）
- 数据链路：X4 → USB → Windows → `scripts/bind-camera-to-wsl.ps1`（usbipd-win，复刻串口映射脚本骨架）→ WSL `/dev/video0` → OpenCV（panthera conda）→ 多模态模型服务（建议 Windows 侧 FastAPI `POST /predict`，返回 `{label, bbox, score}`）→ 坐标变换 → SDK 抓取流程（API-005/006）
- 抓取策略：单目无深度，look-then-move（停稳→拍帧→检测→平面假设求 3D→IK→moveJ 预抓位→moveL 下降），不做实时伺服规避延迟；支架拆装后手眼矩阵失效须重标定（`handeye_x4.yaml`），内参文件 `calib_x4.yaml`（鱼眼 `cv2.fisheye.calibrate`，全景档需先做局部透视投影）
- 文档：安装需求书（可转交协助方）`docs/insta/insta360-x4-mount-requirements.md`；分阶段调试路线 `docs/insta/insta360-x4-integration-guide.md`（Phase 0 接口确认 / 1 捕获与模型接口 / 2 WSL 映射权限 / 3 内参标定 / 4 手眼标定+触点验证≤5mm / 5 抓取闭环 / 6 安装注意事项）
- 常见坑：Windows 侧应用（相机应用/OBS/会议软件）占用相机时 WSL 看不到设备；usbipd attach 为临时绑定，WSL 重启后需重新执行；权限不足 `sudo usermod -aG video $USER`

### [VIS-005] 腕部安装约束与物理注意事项（Insta360 X4）
- 重量：相机+支架约 250g（X4 约 203g 含电池），远低于 3.5kg 负载上限，但改变末端质心，视觉任务限速 0.3~0.5 rad/s
- 运动模糊：必须停稳再拍（`wait_for_position` 后延迟 100~200ms 消振再取帧）
- 散热：X4 长时间 Webcam 模式发热大、可能过温自动停机；上线前实测连续 30 分钟出流；脚本须检测 `cap.read()` 失败并显式报错提示重连，不得静默继续
- 走线：单根 USB-C 线沿臂身固定，J4、J5 关节处预留松弛环，保证全关节行程不受拉不绞线；禁止从机械臂 24V 主电源直接取电（如需独立供电用 5V DC-DC 隔离）
- 安装位置：腕部 J6 法兰附近，主工作镜头朝向夹爪作业方向、另一镜头背向夹爪开口区；支架与夹爪开合包络及工件抓取区零干涉；支架螺栓点胶防松

---

## 8. 安全与免责

### [SAFE-001] 安全规范
- 掉电必须扶住机械臂防止跌落（官方 CAUTION）
- 真机模式发送指令/运行脚本前确认周围无人和障碍物
- 真机脚本会直接控制硬件，运行前先查看脚本内容
- 轨迹/waypoint 不得穿过桌面、夹具或人体附近
- 从补偿模式切回位置模式由后端自动平滑回零，仍需现场监护
