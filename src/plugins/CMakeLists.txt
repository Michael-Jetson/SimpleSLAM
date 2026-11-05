# 插件层：算法契约与具体算法（三分法里的"插件"）。
#
# 纯算法，按帧调用，不碰总线、不碰资源、无生命周期。依赖方向是单向的：
# 插件依赖框架契约（数据类型），框架不依赖插件，模块同时依赖两者。
# 写算法的人只在这个目录里加文件——契约在 include/SimpleSLAM/plugins/，
# 需要实现文件的（如数据集读取器）放 src/plugins/src/。
add_library(simpleslam_plugins STATIC
    src/kitti_source.cpp
)
target_include_directories(simpleslam_plugins PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_plugins PUBLIC cxx_std_20)
target_link_libraries(simpleslam_plugins PUBLIC simpleslam_framework)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_plugins PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::plugins ALIAS simpleslam_plugins)
