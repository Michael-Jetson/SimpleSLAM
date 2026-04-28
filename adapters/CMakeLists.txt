# simpleslam_adapters —— 类型转换桥接层（header-only）
# manif::SE3d ↔ gtsam::Pose3 等外部库类型转换

add_library(simpleslam_adapters INTERFACE)

target_include_directories(simpleslam_adapters
    INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam_adapters
    INTERFACE
        simpleslam_core
)

if(SIMPLESLAM_USE_GTSAM)
    find_package(GTSAM REQUIRED)
    target_link_libraries(simpleslam_adapters INTERFACE gtsam)
    target_compile_definitions(simpleslam_adapters INTERFACE SIMPLESLAM_HAS_GTSAM)
endif()
