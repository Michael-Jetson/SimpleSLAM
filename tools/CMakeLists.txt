# 命令行工具。simpleslam-eval 评测别的系统跑出来的轨迹，用不到用户写的实现，所以单独成一个
# 可执行文件；跑任务、配置命令、补评都经用户自己的入口，写法见 examples/。
add_executable(simpleslam_eval_tool eval_tool.cpp)
set_target_properties(simpleslam_eval_tool PROPERTIES OUTPUT_NAME simpleslam-eval)
target_link_libraries(simpleslam_eval_tool PRIVATE simpleslam_app)

if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_eval_tool PRIVATE simpleslam_compiler_options)
endif()
