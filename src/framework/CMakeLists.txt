# 框架契约层：时间与时钟域、传感器数据、数据包、事件、资源、算法接口。
# 目前全部是头文件；有非模板实现时在此添加 .cpp。
add_library(simpleslam_framework INTERFACE)
target_include_directories(simpleslam_framework INTERFACE "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_framework INTERFACE cxx_std_20)
target_link_libraries(simpleslam_framework INTERFACE
    simpleslam_comm
    Eigen3::Eigen
    ${SIMPLESLAM_MANIF_TARGET}
)
add_library(SimpleSLAM::framework ALIAS simpleslam_framework)
