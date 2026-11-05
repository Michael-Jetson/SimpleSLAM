cmake_minimum_required(VERSION 3.20)

project(SimpleSLAM
    VERSION 1.1
    DESCRIPTION "模块化 C++20 SLAM 框架"
    LANGUAGES CXX
)
# 版本号分两级 X.Y：大改动升大版本，小改动升小版本。每次合进 main 定一次版本，
# 与 git 标签 vX.Y 一致。

# ── C++20 全局设置 ──
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── 加载自定义 CMake 模块 ──
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CompilerOptions)

# ── 构建范围 ──
# 新栈 = 通信运行时 + 框架契约 + 模块层（只需要 Eigen / manif / Catch2）。
# 旧垂直（core/sensor_io/odometry/backend/runner）另需 nanoflann 与 robin-map，
# 在依赖不全的环境里可以只构建新栈：cmake -DSIMPLESLAM_NEW_STACK_ONLY=ON
option(SIMPLESLAM_NEW_STACK_ONLY "只构建通信/框架/模块新栈" OFF)

# ── 查找依赖 ──
find_package(Eigen3 3.4 REQUIRED)

# manif 的 CMake 配置文件不一定随头文件一起安装（例如只 COPY 头文件的镜像）
find_package(manif QUIET)
if(TARGET MANIF::manif)
    set(SIMPLESLAM_MANIF_TARGET MANIF::manif)
else()
    find_path(SIMPLESLAM_MANIF_INCLUDE_DIR manif/SE3.h REQUIRED)
    add_library(simpleslam_manif_headers INTERFACE)
    target_include_directories(simpleslam_manif_headers SYSTEM INTERFACE
        "${SIMPLESLAM_MANIF_INCLUDE_DIR}")
    set(SIMPLESLAM_MANIF_TARGET simpleslam_manif_headers)
    message(STATUS "manif: 未找到 CMake 配置，按头文件路径使用 (${SIMPLESLAM_MANIF_INCLUDE_DIR})")
endif()

# yaml-cpp：新栈的配置、精度基线与实验回放包都用它。旧栈里它是硬依赖，
# 新栈里是软依赖——没有它就不构建评测/回放/配置三层，其余照常。
find_package(yaml-cpp QUIET)
if(TARGET yaml-cpp::yaml-cpp)
    set(SIMPLESLAM_YAMLCPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(SIMPLESLAM_YAMLCPP_TARGET yaml-cpp)
endif()

# 评测 / 回放 / 配置三层。写算法的人要的"跑完能看到精度、能重来一遍"在这里。
option(SIMPLESLAM_BUILD_TOOLING "构建评测/回放/配置层（需要 yaml-cpp）" ON)
if(SIMPLESLAM_BUILD_TOOLING AND NOT DEFINED SIMPLESLAM_YAMLCPP_TARGET)
    message(STATUS "未找到 yaml-cpp：评测/回放/配置层不构建")
    set(SIMPLESLAM_BUILD_TOOLING OFF)
endif()

if(NOT SIMPLESLAM_NEW_STACK_ONLY)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(nanoflann REQUIRED)
find_package(tsl-robin-map REQUIRED)
# eventpp 和 SPSCQueue 预留给 v1.5 online 高性能模式
# v1.0 的 Topic<T> 用标准库实现全部三种 QoS，接口完全兼容

if(NOT DEFINED SIMPLESLAM_YAMLCPP_TARGET)
    message(FATAL_ERROR "旧栈需要 yaml-cpp，但没有找到它的 CMake target")
endif()

# spdlog 目标名兼容
if(TARGET spdlog::spdlog)
    set(SIMPLESLAM_SPDLOG_TARGET spdlog::spdlog)
elseif(TARGET spdlog::spdlog_header_only)
    set(SIMPLESLAM_SPDLOG_TARGET spdlog::spdlog_header_only)
else()
    message(FATAL_ERROR "找不到 spdlog 的 CMake target")
endif()

endif()  # NOT SIMPLESLAM_NEW_STACK_ONLY

# ── 可选依赖 ──
option(SIMPLESLAM_USE_GTSAM "启用 GTSAM 集成" OFF)


# ── 源码（各模块库，实现在 src/，公共头文件在 include/）──
add_subdirectory(src)

# ── 命令行工具（配置 schema 导出、轨迹评测）与示例入口 ──
if(SIMPLESLAM_BUILD_TOOLING)
    add_subdirectory(tools)
    add_subdirectory(examples)
endif()

# ── 应用入口（L1 配置驱动可执行）──
if(NOT SIMPLESLAM_NEW_STACK_ONLY)
    add_subdirectory(apps)
endif()

# ── 测试（默认开启）──
option(BUILD_TESTING "构建单元测试" ON)
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# ── 文档校验 ──
# 内部文档和检查脚本只在本地，不随仓库发布。检出里没有它们时（公开仓库、Docker 的
# verify 阶段：.dockerignore 排除了 docs/），这个目标只报一声跳过，不让构建失败。
if(EXISTS "${PROJECT_SOURCE_DIR}/scripts/doc_lint.py" AND EXISTS "${PROJECT_SOURCE_DIR}/docs/README.md")
    add_custom_target(docs-verify
        COMMAND ${CMAKE_COMMAND} -E env python3 ${PROJECT_SOURCE_DIR}/scripts/doc_lint.py
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "文档检查"
    )
else()
    add_custom_target(docs-verify
        COMMAND ${CMAKE_COMMAND} -E echo "docs-verify：这份检出里没有内部文档，跳过"
    )
endif()
