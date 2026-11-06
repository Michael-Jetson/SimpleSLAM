# 通信运行时。
#
# 两种用法：
#   1) 作为主项目的子目录（add_subdirectory）；
#   2) 独立配置：cmake -S src/comm -B <build>，只需要 Catch2，
#      不需要 manif 与 Eigen，用于快速迭代通信层本身（日志库一并编进来）。
cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(SimpleSLAMComm LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    set(SIMPLESLAM_COMM_STANDALONE_PROJECT ON)
    include(CTest)
else()
    set(SIMPLESLAM_COMM_STANDALONE_PROJECT OFF)
endif()

set(SIMPLESLAM_COMM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../include")

find_package(Threads REQUIRED)

# 模块的日志对象来自日志库；独立配置时从这里把它加进来
if(NOT TARGET simpleslam_logging)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../logging" "${CMAKE_CURRENT_BINARY_DIR}/logging")
endif()

add_library(simpleslam_comm STATIC
    src/clock.cpp
    src/core.cpp
    src/mailbox.cpp
    src/module.cpp
    src/names.cpp
    src/parameters.cpp
    src/runtime.cpp
    src/service.cpp
    src/topic.cpp)
target_include_directories(simpleslam_comm PUBLIC "${SIMPLESLAM_COMM_INCLUDE_DIR}")
target_compile_features(simpleslam_comm PUBLIC cxx_std_20)
target_link_libraries(simpleslam_comm PUBLIC simpleslam_logging Threads::Threads)

if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_comm PRIVATE simpleslam_compiler_options)
else()
    target_compile_options(simpleslam_comm PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(SimpleSLAM::comm ALIAS simpleslam_comm)

# ── 测试（独立配置时才在这里建；作为子目录时由 tests/CMakeLists.txt 统一登记）──
if(SIMPLESLAM_COMM_STANDALONE_PROJECT AND BUILD_TESTING)
    find_package(Catch2 3 REQUIRED)
    set(SIMPLESLAM_COMM_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/unit")
    foreach(t comm_names comm_topic comm_params comm_service comm_runtime comm_lifecycle comm_module)
        add_executable(test_${t} "${SIMPLESLAM_COMM_TEST_DIR}/test_${t}.cpp")
        target_link_libraries(test_${t} PRIVATE simpleslam_comm Catch2::Catch2WithMain)
        target_compile_options(test_${t} PRIVATE -Wall -Wextra -Wpedantic)
        add_test(NAME test_${t} COMMAND test_${t})
    endforeach()
endif()
