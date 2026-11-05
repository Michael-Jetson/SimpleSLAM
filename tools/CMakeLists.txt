# 命令行工具。存在的理由是让 C++ 里的真源（配置 schema、评测口径）能被
# 脚本与 CI 直接使用，而不是在 Python 里再实现一遍。
add_executable(simpleslam_config_tool config_tool.cpp)
set_target_properties(simpleslam_config_tool PROPERTIES OUTPUT_NAME simpleslam-config)
target_link_libraries(simpleslam_config_tool PRIVATE simpleslam_config)

add_executable(simpleslam_eval_tool eval_tool.cpp)
set_target_properties(simpleslam_eval_tool PROPERTIES OUTPUT_NAME simpleslam-eval)
target_link_libraries(simpleslam_eval_tool PRIVATE simpleslam_eval)

if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_config_tool PRIVATE simpleslam_compiler_options)
    target_link_libraries(simpleslam_eval_tool PRIVATE simpleslam_compiler_options)
endif()
