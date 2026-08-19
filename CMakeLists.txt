cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(go2_deploy)

SET(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# ---------------------------------------------------------------
# onnxruntime (vendored C++ SDK in third_party/onnxruntime)
#   lib/<arch>/ : arch-specific prebuilt libonnxruntime.so (x86_64 or arm64)
#   include/    : arch-independent headers
# ---------------------------------------------------------------
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/onnxruntime")

set(ONNXRUNTIME_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT ONNXRUNTIME_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE ONNXRUNTIME_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(ONNXRUNTIME_ARCH MATCHES "^(AMD64|amd64|x86_64)$")
    set(ONNXRUNTIME_ARCH x86_64)
elseif(ONNXRUNTIME_ARCH MATCHES "^(aarch64|arm64)$")
    set(ONNXRUNTIME_ARCH arm64)
else()
    message(FATAL_ERROR "Unsupported onnxruntime architecture: ${ONNXRUNTIME_ARCH}")
endif()
message(STATUS "onnxruntime architecture: ${ONNXRUNTIME_ARCH}")

set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}/libonnxruntime.so")
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
    message(FATAL_ERROR "onnxruntime library not found: ${ONNXRUNTIME_LIBRARY}")
endif()

include_directories(
    include
    ${ONNXRUNTIME_DIR}/include)

add_executable(go2_deploy 
src/main.cpp)

target_link_libraries(go2_deploy
    unitree_sdk2 
    yaml-cpp
    ${ONNXRUNTIME_LIBRARY}
    pthread
    ${CMAKE_DL_LIBS}
    )

# make libonnxruntime.so findable at runtime
set_target_properties(go2_deploy PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    )
