# 设计文档：深度图越障（Parkour）策略集成

- 日期：2026-09-01
- 仓库：`/home/lupinjia/unitree_robotics/go2_deploy`（分支 `dev_ort`）
- 状态：**已获用户批准**（架构、状态接入、深度管线、4 个子项全部确认）

## 1. 目标

将 mjlab 训练的**深度图越障（parkour）策略**集成进 go2_deploy，使其在一个**独立的
状态机状态（PARKOUR）**中运行。当前阶段为 **sim2sim**：MuJoCo（unitree_mujoco，C++
路径）通过 DDS 发布深度图，go2_deploy 接收并推理。部署参数**完全由 ONNX metadata
驱动**（与现有盲策略一致）。

示例模型：`models/2026-09-01_11-32-56_parkour_cts_depth_9000.onnx`

## 2. 背景与关键事实（探索结论）

### 2.1 Parkour 模型结构（已从 ONNX 元数据 + 图结构确认）

- **4 输入 / 2 输出**：
  - `current_obs`  [1, 45] — 当前 obs，term-major，无历史
  - `obs_history`  [1, 900] — 每项 20 帧历史的 term-major 拼接（6 项 × 45 = 900）
  - `depth_image`  [1, 2, 30, 53] — 2 帧深度历史堆到 channel 轴（旧→新）
  - `hidden_state` [1, 1, 512] — GRU 循环状态
  - 输出：`actions` [1, 12]，`new_hidden_state` [1, 1, 512]
- **RNN**：GRU，512 hidden，1 层（`rnn_type=gru, rnn_hidden_dim=512, rnn_num_layers=1`）
- **深度 CNN 已内嵌**（`depth_cnn`：Conv→Flatten→Gemm→Elu→Gemm），无独立权重文件
- **深度相机参数**（metadata）：`depth_image_height=30` `depth_image_width=53`
  `depth_fovy=58` `depth_far_plane=5.0` `depth_cutoff_distance=2.5` `depth_min_depth=0.105`

### 2.2 Obs 布局（已对照 mjlab ObservationManager 验证）

- 预处理顺序：compute → noise → clip → scale → history；**term-major** 拼接
- **current_obs [1,45]**：base_ang_vel(3,×0.25)、projected_gravity(3,×1.0)、
  vel_command(3,×(1,1,0.25))、joint_pos(12,×1.0, 相对 default)、joint_vel(12,×0.05)、
  actions(12,×0.2)。**先乘 scale 再送模型**（模型内部另有 running normalization）
- **obs_history [1,900]**：每项 20 帧历史 term-major
  `[base_ang_vel(60), projected_gravity(60), vel_command(60), joint_pos(240), joint_vel(240), actions(240)]`，
  首帧回填（非补零），旧→新
- **depth_image [1,2,30,53]**：2 帧，channel 0=旧，channel 1=新；每帧
  `clamp(depth_m, 0.105, 2.5)/2.5` 裁剪到 [0,1]（训练侧 `camera_depth` 定义）
- **hidden_state**：初始 0，每步回写 `new_hidden_state`

### 2.3 DDS 深度图（参考 test_scripts）

- Topic：`rt/depthimage`；消息类型：`unitree_go::msg::dds_::DepthImage_`
  （`uint16 width` + `uint16 height` + `std::vector<float> normalized_value`）
- go2_deploy **当前没有此消息类型**，需从
  `/home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/` 拷贝
  `DepthImage_.hpp/.cpp`（Cyclone DDS v0.10.2 生成）到 `include/msg/`
- 订阅：`ChannelSubscriber<DepthImage_>("rt/depthimage")`，`InitChannel(handler, 1)`
  （queue=1 最新帧），`ChannelFactory::Instance()->Init(domain, interface)`

### 2.4 MuJoCo 发布端（unitree_mujoco）

- C++ 路径：`simulate/src/utils/depth_display.hpp`（编译进 `unitree_mujoco` 二进制，
  由 `main.cc` 实例化），`enable_depth: 0` 默认关闭
  - 当前：320×240 渲染，crop_left=8 → 发布 312×240，归一化 `(d-0.175)/(10.0-0.175)`
  - **与训练不匹配**（模型要求 30×53 + clamp(d,0.105,2.5)/2.5）
- Python 路径（`simulate_python`）：80×60，自定义 `unitree_mujoco.DepthImage_`，
  **与 C++ 类型不兼容** —— 不使用

## 3. 设计决策（用户已确认）

| 决策点 | 选择 |
|--------|------|
| 控制器架构 | **新增独立 `Go2ParkourController` 类**，与 `Go2OnnxController` 共存 |
| 状态接入 | **新增 `STATES::PARKOUR`**，`L1 + X` 进入、`L1 + Y` 停止 |
| 深度管线 | **改 MuJoCo C++ `DepthDisplay` 对齐训练**：53×30 + `clamp(d,0.105,2.5)/2.5` |
| 深度 resize 实现 | **引入 OpenCV**（go2_deploy 依赖 opencv4） |
| 深度订阅配置 | **yaml 可配置**（domain/interface/topic） |
| 部署配置位置 | **复用 `go2_config.yaml` 加字段** |

## 4. 架构

```
MuJoCo C++ DepthDisplay (53×30, clamp(d,0.105,2.5)/2.5)  [改]
        │  DDS rt/depthimage (unitree_go::msg::dds_::DepthImage_)
        ▼
DepthImageSubscriber (go2_deploy)  [新]  →  2帧环形缓冲 [2,30,53]
        │
Go2ParkourController (PARKOUR 状态)  [新]
    ├─ current_obs   [1,45]   term-major 无历史（先乘 scale）
    ├─ obs_history   [1,900]  term-major 20帧/项（首帧回填）
    ├─ depth_image   [1,2,30,53]  2帧 channel 轴（旧→新）
    └─ hidden_state  [1,1,512] GRU 状态, 每步回写
```

## 5. 组件设计

### 5.1 `include/onnx_metadata.hpp`（扩展）

在现有 `OnnxMetadata` 上**新增**字段与解析（保持 base keys 向后兼容）：

- 多输入描述：`onnx_input_names` / `onnx_input_shapes` / `onnx_input_groups` /
  `onnx_output_names` / `onnx_output_shapes`
  - 形状 dims 以 float 序列化（如 `45.000`），解析时转 int
- 分组结构：`obs_group.<g>.*`：
  - `history_length`(int)、`flatten_history_dim`(bool)、`term_names`(list[str])、
    `term_history_lengths`(list[int])、`term_clips`(list[list])、`term_scales`(list)
  - `term_scales` 每条可为**标量或 `;` 分隔的 per-dim 列表**
- 深度相机：`depth_image_height` / `depth_image_width` / `depth_fovy` /
  `depth_far_plane` / `depth_cutoff_distance` / `depth_min_depth`
- RNN：`rnn_type` / `rnn_hidden_dim` / `rnn_num_layers`
- 提供便捷访问：`has_perception()`（是否含 `onnx_input_groups`）、
  `input_shape(name)`、`obs_group(name)`、`depth_width()` 等

### 5.2 `include/observation_builder.hpp`（扩展）

现有单向量 `compute()`（盲策略）**保留不动**。新增：

- `compute_current()` → `std::vector<float>` [45]，term-major 无历史，先乘 scale
- `compute_history()` → `std::vector<float>` [900]，每项 20 帧历史 term-major
- 深度帧缓冲：`set_depth_frame(const float* data, size_t n)`（30×53），维护 2 帧
  deque（旧→新），`depth_input()` 返回 [2,30,53] 的 flat vector
- 复用现有 `compute_raw`/`compute_term`/`push_history` 机制（按 term 名查 metadata
  的 scale/clip/history）

**term 名差异（关键）**：现有盲策略模型 `go2_rough.onnx` 的 obs term 名为 `command`
（`observation_names: command,base_ang_vel,...`），而 parkour 模型为 `vel_command`
（`observation_names: base_ang_vel,projected_gravity,vel_command,...`，且
`obs_scale.vel_command` 为 per-dim 列表 `[1,1,0.25]`）。当前 `compute_raw()` 只实现了
`command`。**必须把 `vel_command` 作为 `command` 的别名**：`compute_raw()` 中
`name == "command" || name == "vel_command"` 均返回 `command_` 向量。同理
`compute_raw_dim()` 需对 `vel_command` 返回 3。

### 5.3 `include/user_controller.hpp`（新增 `Go2ParkourController`）

独立类，实现 `BasicUserController` 虚接口：

- `loadParam`：同现有，读 yaml
- `loadPolicy`：4 输入 / 2 输出 session。按 metadata 的 `onnx_input_names/shapes`
  构造 tensor，`onnx_output_names` 取 actions + new_hidden_state。joint 配置/
  PD 映射复用现有逻辑（`meta2real`）
- `reset`：清空 hidden_state（零）、深度缓冲、obs 历史、last_action
- `Calculate`：
  1. `current = obs_builder_->compute_current()`（[45]）
  2. `history = obs_builder_->compute_history()`（[900]）
  3. `depth = obs_builder_->depth_input()`（[2,30,53]）
  4. 构造 4 个 `Ort::Value` → `session_->Run` → actions + new_hidden_state
  5. 回写 hidden_state、last_action；`jpos_des = stand_pos_real + action*scale`（real 序）
- `GetInput`：从 gamepad 写 `vel_command`（同现有约定）
- `DummyCalculate`：全零 obs + 零 hidden 的 warmup 推理
- 持有 `DepthImageSubscriber*`（注入，非拥有）以读取最新深度帧

### 5.4 `include/depth_image_subscriber.hpp`（新增）

- vendored 消息：`include/msg/DepthImage_.hpp` / `DepthImage_.cpp`
- 构造：`DepthImageSubscriber(domain, interface, topic)` → `ChannelFactory::Init` +
  `ChannelSubscriber<DepthImage_>`
- 回调：`static_cast<const DepthImage_*>(data)`，校验宽高 53×30，memcpy
  `normalized_value` 到内部缓冲（发布侧已归一化，接收侧零变换）
- 线程安全：mutex 保护最新帧；`latest_frame()` 返回最近深度帧指针 + 时间戳
- queue=1（最新帧优先）

### 5.5 `include/state_machine.hpp`（扩展）

- `STATES` 枚举加 `PARKOUR = 4`
- `RLStateMachine` 加 `Parkour()`：仅 `state == STAND` 时可进入（与 Ctrl 平行）

### 5.6 `include/robot_controller.hpp`（扩展）

- `UpdateStateMachine`：`L1 + X` → `state_machine.Parkour()` → `ParkourCallback()`
  - `L1 + Y` 已有 Stop（回 DAMPING），语义一致，复用
- `ParkourCallback()`：`ctrl->reset(...)`、置 stand gains、`jpos_des = stand_pos_real`、
  打印 "Parkour enabled."
- `ControlStep`：`state == STATES::PARKOUR` 分支调 `UserControlStep(true)` +
  `CheckTermination()`
- 持有 `DepthImageSubscriber` 实例，构造后把指针注入 `Go2ParkourController`
- DDS 深度订阅在 `StartControl` 前初始化（与 lowcmd/lowstate 并列）

### 5.7 MuJoCo 侧改造（unitree_mujoco，`simulate/`）

- `config.yaml`：`depth_width: 53` `depth_height: 30` `crop_left: 0` `enable_depth: 1`
- `depth_display.hpp` / `simulate.cc`：渲染仍可保留高分辨率 sub-rect，发布前
  `cv::resize` 到 53×30，归一化改为训练一致 `clamp(depth_m, 0.105, 2.5)/2.5`

### 5.8 构建（CMakeLists.txt）

- 新增源文件：`include/msg/DepthImage_.cpp`、新控制器/订阅器（header-only 则无需）
- 新增依赖：`find_package(OpenCV REQUIRED)` + `target_link_libraries(opencv_* 或
  opencv4::opencv_core opencv4::opencv_imgproc)`

### 5.9 配置（params/go2_config.yaml）

复用现有 yaml，新增字段：

```yaml
# --- parkour / depth ---
policy_name: "2026-09-01_11-32-56_parkour_cts_depth_9000.onnx"  # 部署时切换
depth_topic: "rt/depthimage"
depth_domain: 1          # sim2sim; 实机 0
depth_interface: "lo"    # sim2sim; 实机 e.g. eth0
depth_enabled: true
```

> 说明：`policy_name` 切换盲策略/parkour 模型；控制器类型由
> `onnx_input_groups` 存在性自动选择（`Go2ParkourController` vs `Go2OnnxController`）。

## 6. 数据流

```
ControlStep (50Hz):
  InteprateGamePad() → UpdateStateMachine()
  state == PARKOUR:
    ctrl->GetInput(robot, gamepad)     # vel_command
    ctrl->Calculate()                  # 内部: current+history+depth+hidden → Run → actions
    jpos_des = stand_pos + action*scale
set_cmd() → 写 lowcmd (500Hz 线程)
```

深度订阅回调（DDS 线程，~10Hz sim）：写入 2 帧缓冲 → Calculate 时读取。

## 7. 错误处理

- 深度图不可用（无订阅/超时）：`Calculate` 用最近一帧（stale）或全 1.0（far plane）
  填充，并打印告警（不 abort）
- 尺寸不匹配：回调校验 width/height == 53/30，不符则丢弃该帧
- 模型加载失败 / metadata 缺关键键：`abort`（与现有约定一致）
- RNN 状态：进入 PARKOUR 时 reset；若推理抛异常，回 DAMPING

## 8. 测试

1. **构建**：x86 编译通过，`lsp_diagnostics` 干净
2. **模型加载**：parkour 模型 metadata 解析正确（4 输入/2 输出/深度参数/RNN）
3. **sim2sim 集成测试**：启动 unitree_mujoco（enable_depth:1）+ go2_deploy
   （domain 1/lo），验证：
   - 深度帧以 53×30 到达并进入 2 帧缓冲
   - L1+X 进入 PARKOUR 状态，推理不崩溃，jpos_des 更新
   - 机器人站立并按策略输出动作（视觉验证）
4. **盲策略回归**：切回 `go2_rough.onnx`，确认现有 CTRL 流程不受影响
5. **实机（后续）**：深度相机接入、domain 0 + 网卡

## 9. 范围外（后续）

- 实机深度相机驱动（Go2 自带深度传感器接入）
- 深度图点云/障碍物可视化
- 多策略热切换 UI
