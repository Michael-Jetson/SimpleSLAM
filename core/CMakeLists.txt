# simpleslam_core —— 核心基础设施库
# 纯 C++20，依赖 Eigen + manif + spdlog + yaml-cpp + fmt
# v0 阶段只包含类型定义和基础设施（日志、配置、计时、时钟）

add_library(simpleslam_core
    src/logger.cpp
    src/timing.cpp
    src/config.cpp
)

target_compile_features(simpleslam_core PUBLIC cxx_std_20)

target_include_directories(simpleslam_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_core
    PUBLIC
        Eigen3::Eigen
        MANIF::manif
        ${SIMPLESLAM_SPDLOG_TARGET}
        ${SIMPLESLAM_YAMLCPP_TARGET}
    PRIVATE
        fmt::fmt                      # 仅 .cpp 中使用，不暴露给下游
        simpleslam_compiler_options
)
