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

# ── v0.5 测试 ──

add_executable(test_sensor_data unit/test_sensor_data.cpp)
target_link_libraries(test_sensor_data PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_point_ops unit/test_point_ops.cpp)
target_link_libraries(test_point_ops PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_voxel_grid unit/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_kdtree unit/test_kdtree.cpp)
target_link_libraries(test_kdtree PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_scan_utils unit/test_scan_utils.cpp)
target_link_libraries(test_scan_utils PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_kitti_source unit/test_kitti_source.cpp)
target_link_libraries(test_kitti_source PRIVATE simpleslam_sensor_io Catch2::Catch2WithMain)
target_compile_definitions(test_kitti_source PRIVATE
    SIMPLESLAM_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}"
)

# ── v1.0 测试 ──

add_executable(test_callback_slot unit/test_callback_slot.cpp)
target_link_libraries(test_callback_slot PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_topic unit/test_topic.cpp)
target_link_libraries(test_topic PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_topic_hub unit/test_topic_hub.cpp)
target_link_libraries(test_topic_hub PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_event_types unit/test_event_types.cpp)
target_link_libraries(test_event_types PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_pose_graph unit/test_pose_graph.cpp)
target_link_libraries(test_pose_graph PRIVATE simpleslam_resources Catch2::Catch2WithMain)

add_executable(test_keyframe_store unit/test_keyframe_store.cpp)
target_link_libraries(test_keyframe_store PRIVATE simpleslam_resources Catch2::Catch2WithMain)

add_executable(test_trajectory unit/test_trajectory.cpp)
target_link_libraries(test_trajectory PRIVATE simpleslam_resources Catch2::Catch2WithMain)

add_executable(test_current_state unit/test_current_state.cpp)
target_link_libraries(test_current_state PRIVATE simpleslam_resources Catch2::Catch2WithMain)

add_executable(test_extrinsics_manager unit/test_extrinsics_manager.cpp)
target_link_libraries(test_extrinsics_manager PRIVATE simpleslam_resources Catch2::Catch2WithMain)

add_executable(test_pcd_io unit/test_pcd_io.cpp)
target_link_libraries(test_pcd_io PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_submap unit/test_submap.cpp)
target_link_libraries(test_submap PRIVATE simpleslam_backend Catch2::Catch2WithMain)

# ── 注册到 CTest ──
include(CTest)
include(Catch)
catch_discover_tests(test_geometry)
catch_discover_tests(test_config)
catch_discover_tests(test_sensor_data)
catch_discover_tests(test_point_ops)
catch_discover_tests(test_voxel_grid)
catch_discover_tests(test_kdtree)
catch_discover_tests(test_scan_utils)
catch_discover_tests(test_kitti_source)
catch_discover_tests(test_callback_slot)
catch_discover_tests(test_topic)
catch_discover_tests(test_topic_hub)
catch_discover_tests(test_event_types)
catch_discover_tests(test_pose_graph)
catch_discover_tests(test_keyframe_store)
catch_discover_tests(test_trajectory)
catch_discover_tests(test_current_state)
catch_discover_tests(test_extrinsics_manager)
catch_discover_tests(test_pcd_io)
catch_discover_tests(test_submap)

# ── v1.0 框架层测试 ──

add_executable(test_keyframe_selector unit/test_keyframe_selector.cpp)
target_link_libraries(test_keyframe_selector PRIVATE simpleslam_odometry Catch2::Catch2WithMain)

add_executable(test_any_registration_target unit/test_any_registration_target.cpp)
target_link_libraries(test_any_registration_target PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_health_monitor unit/test_health_monitor.cpp)
target_link_libraries(test_health_monitor PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_odometry_base unit/test_odometry_base.cpp)
target_link_libraries(test_odometry_base PRIVATE simpleslam_odometry Catch2::Catch2WithMain)

add_executable(test_offline_runner unit/test_offline_runner.cpp)
target_link_libraries(test_offline_runner PRIVATE simpleslam_runner Catch2::Catch2WithMain)

catch_discover_tests(test_keyframe_selector)
catch_discover_tests(test_any_registration_target)
catch_discover_tests(test_health_monitor)
catch_discover_tests(test_odometry_base)
catch_discover_tests(test_offline_runner)

# ── v1.0+ 工程基础设施测试 ──

add_executable(test_any_loop_detector unit/test_any_loop_detector.cpp)
target_link_libraries(test_any_loop_detector PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_any_pose_graph_optimizer unit/test_any_pose_graph_optimizer.cpp)
target_link_libraries(test_any_pose_graph_optimizer PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_imu_buffer unit/test_imu_buffer.cpp)
target_link_libraries(test_imu_buffer PRIVATE simpleslam_odometry Catch2::Catch2WithMain)

add_executable(test_submap_manager unit/test_submap_manager.cpp)
target_link_libraries(test_submap_manager PRIVATE simpleslam_backend Catch2::Catch2WithMain)

add_executable(test_loop_closure_service unit/test_loop_closure_service.cpp)
target_link_libraries(test_loop_closure_service PRIVATE simpleslam_backend Catch2::Catch2WithMain)

add_executable(test_pgo_service unit/test_pgo_service.cpp)
target_link_libraries(test_pgo_service PRIVATE simpleslam_backend Catch2::Catch2WithMain)

catch_discover_tests(test_any_loop_detector)
catch_discover_tests(test_any_pose_graph_optimizer)
catch_discover_tests(test_imu_buffer)
catch_discover_tests(test_submap_manager)
catch_discover_tests(test_loop_closure_service)
catch_discover_tests(test_pgo_service)

# ── v1.0+ 注册表测试 ──

add_executable(test_registry unit/test_registry.cpp)
target_link_libraries(test_registry PRIVATE simpleslam_odometry Catch2::Catch2WithMain)

catch_discover_tests(test_registry)

# ── PipelineBuilder 测试 ──

add_executable(test_pipeline_builder unit/test_pipeline_builder.cpp)
target_link_libraries(test_pipeline_builder PRIVATE simpleslam_runner Catch2::Catch2WithMain)

catch_discover_tests(test_pipeline_builder)

# ── v1.0 算法层测试 ──

add_executable(test_voxel_hash_target unit/test_voxel_hash_target.cpp)
target_link_libraries(test_voxel_hash_target PRIVATE simpleslam_odometry Catch2::Catch2WithMain)

add_executable(test_icp_solver unit/test_icp_solver.cpp)
target_link_libraries(test_icp_solver PRIVATE simpleslam_core Catch2::Catch2WithMain)

add_executable(test_lo_icp_odometry unit/test_lo_icp_odometry.cpp)
target_link_libraries(test_lo_icp_odometry PRIVATE simpleslam_odometry Catch2::Catch2WithMain)

catch_discover_tests(test_voxel_hash_target)
catch_discover_tests(test_icp_solver)
catch_discover_tests(test_lo_icp_odometry)
