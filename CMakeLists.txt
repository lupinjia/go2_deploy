cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
project(go2_deploy)

SET(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
include_directories(include ${TORCH_INCLUDE_DIRS})

set(CMAKE_PREFIX_PATH /home/lupinjia/libtorch)  # Change the path to your LibTorch
find_package(Torch REQUIRED)

add_executable(go2_deploy src/main.cpp)

target_link_libraries(go2_deploy
    unitree_sdk2 
    ${TORCH_LIBRARIES}
    )
