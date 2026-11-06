# 日志：日志对象、来源树与日志组件（后台线程）。只依赖标准库，通信层与插件都建在它上面。
# 通信层独立配置时也从这里加进来，所以先看目标是否已经有了。
if(TARGET simpleslam_logging)
    return()
endif()

find_package(Threads REQUIRED)

add_library(simpleslam_logging STATIC
    src/logger.cpp
    src/router.cpp
)
target_include_directories(simpleslam_logging PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
target_compile_features(simpleslam_logging PUBLIC cxx_std_20)
target_link_libraries(simpleslam_logging PUBLIC Threads::Threads)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_logging PRIVATE simpleslam_compiler_options)
else()
    target_compile_options(simpleslam_logging PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_library(SimpleSLAM::logging ALIAS simpleslam_logging)
