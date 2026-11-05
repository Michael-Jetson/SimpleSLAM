# 模块层：把插件（算法）接到总线与资源上的管道
# （传感器接入、里程计、位置识别、图优化、轨迹收集、装配根）。
add_library(simpleslam_modules INTERFACE)
target_include_directories(simpleslam_modules INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_modules INTERFACE cxx_std_20)
target_link_libraries(simpleslam_modules INTERFACE simpleslam_framework simpleslam_plugins)
add_library(SimpleSLAM::modules ALIAS simpleslam_modules)
