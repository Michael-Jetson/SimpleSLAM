# simpleslam_run —— L1 配置驱动离线入口（蓝图三层使用深度 L1）
# 装配逻辑在 simpleslam_runner（runOfflineApp），此处仅薄壳 main。

add_executable(simpleslam_run simpleslam_run.cpp)

target_link_libraries(simpleslam_run
    PRIVATE
        simpleslam_runner
        simpleslam_compiler_options
)
