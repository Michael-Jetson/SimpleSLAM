# simpleslam_runner —— 运行模式编排器
# OfflineRunner: 离线数据集回放主循环

add_library(simpleslam_runner
    src/offline_runner.cpp
)

target_compile_features(simpleslam_runner PUBLIC cxx_std_20)

target_include_directories(simpleslam_runner
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_runner
    PUBLIC
        simpleslam_core
        simpleslam_sensor_io
        simpleslam_resources
        simpleslam_backend
        simpleslam_odometry
    PRIVATE
        simpleslam_compiler_options
)
