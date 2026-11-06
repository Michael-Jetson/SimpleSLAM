# 评测层：轨迹的关联与重采样、TUM/KITTI 读写、ATE/RPE/KITTI 分段误差、评测函数。
#
# 轨迹本身是框架的数据类型（framework/trajectory.hpp），模块层和插件不依赖本库；
# 需要读写文件、算指标时才链接 simpleslam_eval。
add_library(simpleslam_eval STATIC
    src/association.cpp
    src/trajectory_io.cpp
    src/metrics.cpp
    src/evaluator.cpp
    src/evaluators.cpp
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
