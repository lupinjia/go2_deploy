# go2_deploy

Deployment code of RL policy on Unitree Go2 robot, using policies from [genesis_lr](https://github.com/lupinjia/genesis_lr). This framework is based on the [state machine example from Unitree Doc](https://support.unitree.com/home/zh/developer/LowLevel_Ctrl_Framework) and conducts neural network inference via [LibTorch](https://pytorch.org/).

## Platform

[Nvidia Jetson Orin NX 100Tops](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

## Installation

Below operations should be conducted in a Jetson board.

1. Install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

2. Install [LibTorch with CUDA](https://pytorch.org/)
  ```bash
  wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu118.zip # modify cuda version

  # unzip the file and get libtorch folder
  # the CMAKE_PREFIX_PATH in CMakeLists.txt should be modified according to your installation path of libtorch
  set(CMAKE_PREFIX_PATH /home/username/libtorch/share/cmake/Torch)     # in CMakeLists.txt
  ```

3. Clone this repo and compile
  ```bash
  # clone the repo
  git clone https://github.com/lupinjia/go2_deploy.git
  mkdir build && cd build
  cmake .. && make
  ```

4. Modify the networkInterface in `main.cpp`
  ```bash
  # check network interface name
  ifconfig

  # in main.cpp
  /***** Controller *****/
  std::string networkInterface = "eth0"; // !根据本机ifconfig的结果修改该变量
  ```

## Usage

1. To customize your own RL inference code, you need to create a new class in `include/user_controller.hpp` inheriting `BasicUserController`. An example RLController has been provided, which implements NN inference using basic apis of libtorch and double ended queue.

## Acknowledgement

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)