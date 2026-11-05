# 命令行工具。存在的理由是让 C++ 里的真源（配置 schema、评测口径）能被
# 脚本与 CI 直接使用，而不是在 Python 里再实现一遍。
# simpleslam：任务执行器（内置实现）。用户写的算法经用户自己的入口跑，写法见 examples/。
add_executable(simpleslam_run_tool run_tool.cpp)
set_target_properties(simpleslam_run_tool PROPERTIES OUTPUT_NAME simpleslam)
target_link_libraries(simpleslam_run_tool PRIVATE simpleslam_app)

add_executable(simpleslam_config_tool config_tool.cpp)
set_target_properties(simpleslam_config_tool PROPERTIES OUTPUT_NAME simpleslam-config)
target_link_libraries(simpleslam_config_tool PRIVATE simpleslam_app)

add_executable(simpleslam_eval_tool eval_tool.cpp)
set_target_properties(simpleslam_eval_tool PROPERTIES OUTPUT_NAME simpleslam-eval)
target_link_libraries(simpleslam_eval_tool PRIVATE simpleslam_eval)

if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_run_tool PRIVATE simpleslam_compiler_options)
    target_link_libraries(simpleslam_config_tool PRIVATE simpleslam_compiler_options)
    target_link_libraries(simpleslam_eval_tool PRIVATE simpleslam_compiler_options)
endif()
