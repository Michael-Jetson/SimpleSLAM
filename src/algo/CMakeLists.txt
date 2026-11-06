# 算法库：可替换步骤的接口与内置实现（扫描滤波、配准地图）。
#
# 纯算法，只依赖框架的数据类型（点云、位姿）、日志与参数的读写（comm/parameters.hpp），
# 不依赖通信的运行时与模块。
# 内置实现的目录登记（algo/builtins.hpp）只用到 config/plugin.hpp 这个头文件（加上日志库），
# 所以这里不链接配置库。
add_library(simpleslam_algo STATIC
    src/scan_filter.cpp
    src/map.cpp
    src/voxel_point_map.cpp
)
target_include_directories(simpleslam_algo PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_algo PUBLIC cxx_std_20)
target_link_libraries(simpleslam_algo PUBLIC simpleslam_framework)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_algo PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::algo ALIAS simpleslam_algo)
