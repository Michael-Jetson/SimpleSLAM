# simpleslam_core —— 核心基础设施库
# 类型定义、基础设施（日志/配置/计时/时钟）、数学工具、Topic 通信

add_library(simpleslam_core
    src/logger.cpp
    src/timing.cpp
    src/config.cpp
    src/point_ops.cpp
    src/normal_estimation.cpp
    src/pcd_io.cpp
    src/topic_hub.cpp
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
        nanoflann::nanoflann
        tsl::robin_map
    PRIVATE
        fmt::fmt
        simpleslam_compiler_options
)
