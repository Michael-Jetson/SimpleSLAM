# 框架契约层：时间与时钟域、传感器数据、数据包、事件、资源、轨迹。
# 公开头只放数据、接口和一屏以内的小类；非模板的实现在 src/。
add_library(simpleslam_framework STATIC
    src/calibration.cpp
    src/matrix_checks.cpp
    src/pose_graph.cpp
    src/sensor_source.cpp
    src/trajectory.cpp
)
target_include_directories(simpleslam_framework PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_framework PUBLIC cxx_std_20)
target_link_libraries(simpleslam_framework PUBLIC
    simpleslam_comm
    Eigen3::Eigen
    ${SIMPLESLAM_MANIF_TARGET}
)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_framework PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::framework ALIAS simpleslam_framework)
