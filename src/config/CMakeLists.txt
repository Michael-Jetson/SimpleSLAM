# 配置层：配置聚合体（真源）、YAML 加载/导出、schema 导出。
add_library(simpleslam_config STATIC
    src/loader.cpp
    src/calibration_io.cpp
)
target_include_directories(simpleslam_config PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_config PUBLIC cxx_std_20)
target_link_libraries(simpleslam_config
    PUBLIC simpleslam_framework
    PRIVATE ${SIMPLESLAM_YAMLCPP_TARGET}
)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_config PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::config ALIAS simpleslam_config)
