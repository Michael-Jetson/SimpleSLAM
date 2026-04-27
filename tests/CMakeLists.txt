# SimpleSLAM 单元测试
# 使用 Catch2 3.x 测试框架

find_package(Catch2 3 REQUIRED)

# ── 几何类型测试：验证 manif SE3d 别名和基本运算 ──
add_executable(test_geometry unit/test_geometry.cpp)
target_link_libraries(test_geometry PRIVATE simpleslam_core Catch2::Catch2WithMain)

# ── 配置系统测试：验证 YAML 加载、合并、schema 校验 ──
add_executable(test_config unit/test_config.cpp)
target_link_libraries(test_config PRIVATE simpleslam_core Catch2::Catch2WithMain)

# 测试时需要找到 configs/ 目录，传入项目根路径
target_compile_definitions(test_config PRIVATE
    SIMPLESLAM_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}"
)

# ── 注册到 CTest ──
include(CTest)
include(Catch)
catch_discover_tests(test_geometry)
catch_discover_tests(test_config)
