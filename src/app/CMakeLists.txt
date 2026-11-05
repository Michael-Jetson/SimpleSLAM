# 入口层：任务执行器、内置目录、配置到运行期类型的换算。它是唯一同时认识配置层、
# 模块层与各种实现的地方，那几层因此互不依赖。
add_library(simpleslam_app STATIC
    src/bindings.cpp
    src/catalogs.cpp
    src/replay_reader.cpp
    src/run.cpp
    src/task.cpp
)
target_include_directories(simpleslam_app PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_app PUBLIC cxx_std_20)
target_link_libraries(simpleslam_app PUBLIC
    simpleslam_config
    simpleslam_modules
    simpleslam_algo
    simpleslam_eval
    simpleslam_record
)
# 软件版本进任务档案：结果是哪个版本跑出来的，要能查
target_compile_definitions(simpleslam_app PRIVATE SIMPLESLAM_VERSION="${PROJECT_VERSION}")
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_app PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::app ALIAS simpleslam_app)
