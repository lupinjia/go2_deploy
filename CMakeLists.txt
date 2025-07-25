cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
project(go2_rl_controller)

SET(CMAKE_CXX_STANDARD 17)

include_directories(include thirdparty/include thirdparty/include/ddscxx thirdparty/include/iceoryx/v2.0.2 ${TORCH_INCLUDE_DIRS})
link_directories(lib/aarch64 thirdparty/lib/aarch64) # change the architecture used

set(CMAKE_PREFIX_PATH /home/unitree/.local/lib/python3.8/site-packages/torch)     # change the path to your LibTorch
find_package(Torch REQUIRED)

add_executable(go2_rl_controller src/main.cpp)

target_link_libraries(go2_rl_controller
    unitree_sdk2 
    ddsc 
    ddscxx 
    rt 
    pthread
    ${TORCH_LIBRARIES}
    )
