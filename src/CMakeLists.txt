# SimpleSLAM 源码根 —— 各模块库（实现在此，公共头文件在 ${PROJECT_SOURCE_DIR}/include）
# 按依赖顺序加入：core 在最前，runner 在最后

add_subdirectory(core)
add_subdirectory(sensor_io)
add_subdirectory(resources)
add_subdirectory(adapters)
add_subdirectory(backend)
add_subdirectory(odometry)
add_subdirectory(runner)
