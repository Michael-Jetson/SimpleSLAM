# simpleslam_resources —— 共享资源容器
# 位姿图、关键帧存储、轨迹、外参管理、当前状态

add_library(simpleslam_resources
    src/pose_graph.cpp
    src/keyframe_store.cpp
    src/trajectory.cpp
)

target_compile_features(simpleslam_resources PUBLIC cxx_std_20)

target_include_directories(simpleslam_resources
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_resources
    PUBLIC
        simpleslam_core
    PRIVATE
        simpleslam_compiler_options
)
