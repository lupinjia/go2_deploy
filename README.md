# go2_deploy

Deployment code of RL policy on Unitree Go2 robot, using policies from [genesis_lr](https://github.com/lupinjia/genesis_lr). This framework is based on the [state machine example from Unitree Doc](https://support.unitree.com/home/zh/developer/LowLevel_Ctrl_Framework) and conducts neural network inference via **onnxruntime C++ API**.

Single controller type (`Go2OnnxController`) reads all deployment parameters (joint order, PD gains, default pose, action scale, observation terms/scales/clips/history) from the ONNX model's `metadata_props` (mjlab exporter convention, same as [booster_deploy_cpp](https://github.com/lupinjia/booster_deploy_cpp)), and builds the observation **term-major** via `ObservationBuilder` (compute -> clip -> scale -> history per term).

## Platform

- [Nvidia Jetson Orin NX 100Tops](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- x86_64 PC

## Installation

1. Install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
   ```bash
   git clone https://github.com/unitreerobotics/unitree_sdk2.git
   cd unitree_sdk2/
   mkdir build
   cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
   sudo make install
   ```

2. Install [yaml-cpp](https://github.com/jbeder/yaml-cpp)
   ```bash
   sudo apt install libyaml-cpp-dev
   ```

3. Clone this repo and compile
   ```bash
   # clone the repo
   git clone https://github.com/lupinjia/go2_deploy.git
   mkdir build && cd build
   cmake .. && make
   ```

4. Clone unitree_mujoco and compile (for simulation in mujoco)
   
   1. install mujoco
      ```bash
      sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev

      git clone https://github.com/google-deepmind/mujoco.git
      mkdir build && cd build
      cmake ..
      make -j4
      sudo make install

      sudo apt install libyaml-cpp-dev
      ```
   2. install unitree_mujoco
      ```bash
      git clone https://github.com/lupinjia/unitree_mujoco.git
      cd unitree_mujoco/simulate
      mkdir build && cd build
      cmake ..
      make -j4
      ```

## ONNX model

Place your policy ONNX model in `models/` and set `policy_name` in `params/go2_config.yaml`.
The model must carry mjlab-style `metadata_props` (as used by booster_deploy_cpp):

- `joint_names` / `joint_stiffness` / `joint_damping` / `default_joint_pos`
- `action_scale` (scalar or per-joint)
- `observation_names` / `obs_scale.<term>` / `observation_terms_clip` / `observation_terms_history_length`
- `actor_obs_dim` / `actor_history_length` / `actor_flatten_history_dim`

Only `policy_name`, `dt`, transition PD gains and `sit_pos` are configured in yaml; everything else comes from the model metadata.

## Sim2Sim

1. Start the simulation
   ```bash
   # Start mujoco simulation
   cd unitree_mujoco/simulate/build
   ./unitree_mujoco
   ```

2. Start the controller
   ```bash
   cd go2_deploy/build
   ./go2_deploy
   ```
3. Play with the joystick
   - Common state machine logic:
      - L1 + R1 -> Sit
      - L1 + R2 -> Stand
      - L1 + A -> Ctrl
      - L1 + Y -> Stop

## Sim2Real

```bash
# Check your ethernet interface name
ifconfig
# run with the ethernet interface name
./go2_deploy ethernet_name
```

## Architecture

```
go2_deploy/
├── include/
│   ├── robot_controller.hpp      # 框架：DDS 通道 + 状态机 + 50Hz 控制线程 + 500Hz 命令线程
│   ├── robot_interface.hpp       # Go2RobotInterface + go2_joint_names()（真机关节顺序）
│   ├── user_controller.hpp       # BasicUserController 接口 + Go2OnnxController（ORT 推理）
│   ├── onnx_metadata.hpp         # ONNX metadata 解析（booster_deploy_cpp 移植）
│   ├── observation_builder.hpp   # 观测构建器（term-major，compute -> clip -> scale -> history）
│   ├── gamepad.hpp               # 手柄解析
│   ├── state_machine.hpp         # 状态机：DAMPING -> SIT -> STAND -> CTRL
│   └── conversion.hpp            # 四元数/欧拉角/CRC 工具
├── src/main.cpp
├── params/go2_config.yaml        # 部署配置（policy_name/dt/PD 增益/sit_pos）
├── models/                       # ONNX 策略模型
└── third_party/onnxruntime/      # 本地化 onnxruntime C++ SDK（头文件 + lib）
```

## Tips

1. To customize your own RL inference code, extend the observation terms in `observation_builder.hpp` (each term implements `compute_raw`), keyed by the `observation_names` from the ONNX metadata.
2. It's recommended to first simulate in mujoco and then deploy to real robot to avoid potential collapse.

## Acknowledgement

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/main)
- [booster_deploy_cpp](https://github.com/lupinjia/booster_deploy_cpp) (onnx_metadata / observation_builder / onnxruntime vendoring)
