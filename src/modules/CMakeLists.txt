# 模块层：把插件（算法）接到总线与资源上的管道
# （传感器接入、里程计、位置识别、图优化、轨迹收集、装配根）。
# 公开头只放接口与模板；模块的实现在 src/，改一处实现不必重编所有包含它的翻译单元。
add_library(simpleslam_modules STATIC
    src/graph_optimizer_module.cpp
    src/odometry_module.cpp
    src/place_recognizer_module.cpp
    src/sensor_io_module.cpp
    src/slam_system.cpp
    src/topics.cpp
    src/trajectory_module.cpp
)
target_include_directories(simpleslam_modules PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_modules PUBLIC cxx_std_20)
target_link_libraries(simpleslam_modules PUBLIC simpleslam_framework simpleslam_plugins)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_modules PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::modules ALIAS simpleslam_modules)
