# 评测层：轨迹容器（头文件）、TUM/KITTI 读写、ATE/RPE、精度基线与报告。
#
# Trajectory 本身完全在头文件里，模块层只 include 它即可，不必链接本库；
# 需要读写文件、算指标、比基线时才链接 simpleslam_eval。
add_library(simpleslam_eval STATIC
    src/trajectory_io.cpp
    src/metrics.cpp
    src/baseline.cpp
)
target_include_directories(simpleslam_eval PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_eval PUBLIC cxx_std_20)
target_link_libraries(simpleslam_eval
    PUBLIC simpleslam_framework
    PRIVATE ${SIMPLESLAM_YAMLCPP_TARGET}
)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_eval PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::eval ALIAS simpleslam_eval)
