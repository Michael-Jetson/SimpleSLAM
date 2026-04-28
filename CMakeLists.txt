cmake_minimum_required(VERSION 3.20)

project(SimpleSLAM
    VERSION 0.1.0
    DESCRIPTION "模块化 C++20 SLAM 框架"
    LANGUAGES CXX
)

# ── C++20 全局设置 ──
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── 加载自定义 CMake 模块 ──
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CompilerOptions)

# ── 查找依赖 ──
find_package(Eigen3 3.4 REQUIRED)
find_package(manif REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(nanoflann REQUIRED)
find_package(tsl-robin-map REQUIRED)
# eventpp 和 SPSCQueue 预留给 v1.5 online 高性能模式
# v1.0 的 Topic<T> 用标准库实现全部三种 QoS，接口完全兼容

# yaml-cpp 目标名兼容（0.7.x 导出 yaml-cpp，0.8+ 导出 yaml-cpp::yaml-cpp）
if(TARGET yaml-cpp::yaml-cpp)
    set(SIMPLESLAM_YAMLCPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(SIMPLESLAM_YAMLCPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "找不到 yaml-cpp 的 CMake target")
endif()

# spdlog 目标名兼容
if(TARGET spdlog::spdlog)
    set(SIMPLESLAM_SPDLOG_TARGET spdlog::spdlog)
elseif(TARGET spdlog::spdlog_header_only)
    set(SIMPLESLAM_SPDLOG_TARGET spdlog::spdlog_header_only)
else()
    message(FATAL_ERROR "找不到 spdlog 的 CMake target")
endif()

# ── 可选依赖 ──
option(SIMPLESLAM_USE_GTSAM "启用 GTSAM 集成" OFF)

# ── 子目录 ──
add_subdirectory(core)
add_subdirectory(sensor_io)
add_subdirectory(resources)
add_subdirectory(adapters)
add_subdirectory(backend)

# ── 测试（默认开启）──
option(BUILD_TESTING "构建单元测试" ON)
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
