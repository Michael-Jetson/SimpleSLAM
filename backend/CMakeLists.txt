# simpleslam_backend —— 后端服务框架（v1.0 为 header-only）
# 服务基类、子图数据结构

add_library(simpleslam_backend INTERFACE)

target_include_directories(simpleslam_backend
    INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_backend
    INTERFACE
        simpleslam_core
)
