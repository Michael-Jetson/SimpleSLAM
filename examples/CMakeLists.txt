# 示例：能跑的入口。算法在 lio_skeleton/my_lio.hpp 里，其余是接缝的示范。
add_executable(simpleslam_lio_skeleton lio_skeleton/main.cpp)
set_target_properties(simpleslam_lio_skeleton PROPERTIES OUTPUT_NAME simpleslam-lio-skeleton)
target_link_libraries(simpleslam_lio_skeleton
    PRIVATE simpleslam_modules simpleslam_plugins simpleslam_eval simpleslam_record
            simpleslam_config)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_lio_skeleton PRIVATE simpleslam_compiler_options)
endif()
