# simpleslam_sensor_io —— 传感器数据接入层
# 数据集读取、格式转换、多源合并

add_library(simpleslam_sensor_io
    src/kitti_source.cpp
    src/sensor_mux.cpp
    src/euroc_source.cpp
)

target_compile_features(simpleslam_sensor_io PUBLIC cxx_std_20)

target_include_directories(simpleslam_sensor_io
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_sensor_io
    PUBLIC
        simpleslam_core
    PRIVATE
        simpleslam_compiler_options
)
