# go2_deploy

Deployment code of RL policy on Unitree Go2 robot, using policies from [genesis_lr](https://github.com/lupinjia/genesis_lr). This framework is based on the [state machine example from Unitree Doc](https://support.unitree.com/home/zh/developer/LowLevel_Ctrl_Framework) and conducts neural network inference via [LibTorch](https://pytorch.org/).

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

2. Install [LibTorch](https://pytorch.org/)
   ```bash
   # For Nvidia Jetson
   wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu118.zip # modify cuda version
   # For x86_64 PC
   wget https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.8.0%2Bcpu.zip

   # unzip the file and get libtorch folder
   # the CMAKE_PREFIX_PATH in CMakeLists.txt should be modified according to your installation path of libtorch
   set(CMAKE_PREFIX_PATH /home/username/libtorch)     # in CMakeLists.txt
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

5. Simulate in mujoco
   ```bash
   # Start mujoco simulation
   cd unitree_mujoco/simulate/build
   ./unitree_mujoco
   # Start Controller
   cd ../../../build
   ./go2_deploy
   ```

6. Deploy to real robot
   ```bash
   # Check your ethernet interface name
   ifconfig
   # Start controller
   ./go2_deploy ethernet_name
   ```

## Usage

1. To customize your own RL inference code, you need to create a new class in `include/user_controller.hpp` inheriting `BasicUserController`. An example RLController has been provided, which implements NN inference using basic apis of libtorch and double ended queue.

### Simulate in Mujoco



## Demo

- Tuning robot behavior using [walk these ways](https://github.com/Improbable-AI/walk-these-ways) style
  
  ![](./walk_these_ways.gif)

## Acknowledgement

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/main)