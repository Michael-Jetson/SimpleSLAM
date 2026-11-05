# 入口层：配置 → 运行期类型与装配选项。它是唯一同时认识配置层和模块层的地方，
# 两者因此互不依赖。
add_library(simpleslam_app STATIC
    src/bindings.cpp
)
target_include_directories(simpleslam_app PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_app PUBLIC cxx_std_20)
target_link_libraries(simpleslam_app PUBLIC simpleslam_config simpleslam_modules)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_app PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::app ALIAS simpleslam_app)
