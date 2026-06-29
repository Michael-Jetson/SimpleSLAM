# simpleslam_odometry —— 里程计框架（header-only）
# OdometryBase 抽象基类 + KeyframeSelector 关键帧策略

add_library(simpleslam_odometry INTERFACE)

target_include_directories(simpleslam_odometry
    INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_odometry
    INTERFACE
        simpleslam_core
)
