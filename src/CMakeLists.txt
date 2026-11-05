# SimpleSLAM 源码根 —— 各模块库（实现在此，公共头文件在 ${PROJECT_SOURCE_DIR}/include）
# 按依赖顺序加入：core 在最前，runner 在最后

# 新栈：通信 → 框架契约 → 算法库 → 插件（方案）→ 模块层 → 评测 / 回放 / 配置
add_subdirectory(comm)
add_subdirectory(framework)
add_subdirectory(algo)
add_subdirectory(plugins)
add_subdirectory(modules)

if(SIMPLESLAM_BUILD_TOOLING)
    add_subdirectory(eval)
    add_subdirectory(record)
    add_subdirectory(config)
endif()

if(SIMPLESLAM_NEW_STACK_ONLY)
    return()
endif()

add_subdirectory(core)
add_subdirectory(sensor_io)
add_subdirectory(resources)
add_subdirectory(adapters)
add_subdirectory(backend)
add_subdirectory(odometry)
add_subdirectory(runner)
