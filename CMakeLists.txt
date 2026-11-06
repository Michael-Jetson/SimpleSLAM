cmake_minimum_required(VERSION 3.20)

project(SimpleSLAM
    VERSION 7.1
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

# yaml-cpp：配置与任务记录用它。是软依赖：没有它就不构建评测、配置与执行器，其余照常。
find_package(yaml-cpp QUIET)
if(TARGET yaml-cpp::yaml-cpp)
    set(SIMPLESLAM_YAMLCPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(SIMPLESLAM_YAMLCPP_TARGET yaml-cpp)
endif()

# 评测、配置与执行器。写算法的人要的"跑完能看到精度、能重来一遍"在这里。
option(SIMPLESLAM_BUILD_TOOLING "构建评测、配置与执行器（需要 yaml-cpp）" ON)
if(SIMPLESLAM_BUILD_TOOLING AND NOT DEFINED SIMPLESLAM_YAMLCPP_TARGET)
    message(STATUS "未找到 yaml-cpp：评测、配置与执行器不构建")
    set(SIMPLESLAM_BUILD_TOOLING OFF)
endif()

# ── 源码（各模块库，实现在 src/，公共头文件在 include/）──
add_subdirectory(src)

# ── 命令行工具（轨迹评测）与示例入口 ──
if(SIMPLESLAM_BUILD_TOOLING)
    add_subdirectory(tools)
    add_subdirectory(examples)
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
