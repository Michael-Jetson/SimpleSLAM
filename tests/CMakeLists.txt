# SimpleSLAM 单元测试
# 使用 Catch2 3.x 测试框架

find_package(Catch2 3 REQUIRED)
include(CTest)
include(Catch)

# 契约测试模板与合成来源（include/SimpleSLAM/testing/，只有头文件）。用户的实现在自己的
# 测试里链接它，一行实例化模板
add_library(simpleslam_testing INTERFACE)
target_link_libraries(simpleslam_testing
    INTERFACE simpleslam_modules simpleslam_algo Catch2::Catch2)

# ── 通信运行时 / 框架契约 / 模块层：只依赖 Eigen、manif 与 Catch2 ──
# 日志：来源树、日志组件与后台线程；槽怎么把来源交给实现（config/plugin.hpp 是头文件）
add_executable(test_logging unit/test_logging.cpp)
target_link_libraries(test_logging PRIVATE simpleslam_logging Catch2::Catch2WithMain)
catch_discover_tests(test_logging)

foreach(t comm_names comm_topic comm_params comm_service comm_runtime comm_lifecycle comm_module)
    add_executable(test_${t} unit/test_${t}.cpp)
    target_link_libraries(test_${t} PRIVATE simpleslam_comm Catch2::Catch2WithMain)
    catch_discover_tests(test_${t})
endforeach()

# 几何类型（core/types/geometry.hpp 的 manif 与 Eigen 别名）随框架一起测
foreach(t geometry framework_time framework_resources framework_packetizer)
    add_executable(test_${t} unit/test_${t}.cpp)
    target_link_libraries(test_${t} PRIVATE simpleslam_framework Catch2::Catch2WithMain)
    catch_discover_tests(test_${t})
endforeach()

# 插件槽与目录、算法库：只依赖框架的数据类型
foreach(t config_plugin algo_filter algo_map)
    add_executable(test_${t} unit/test_${t}.cpp)
    target_link_libraries(test_${t} PRIVATE simpleslam_algo Catch2::Catch2WithMain)
    catch_discover_tests(test_${t})
endforeach()

add_executable(test_slam_system unit/test_slam_system.cpp)
target_link_libraries(test_slam_system PRIVATE simpleslam_modules Catch2::Catch2WithMain)
catch_discover_tests(test_slam_system)

# ── 评测 / 配置 / 执行器 ──
if(SIMPLESLAM_BUILD_TOOLING)
    foreach(t eval_trajectory eval_metrics eval_evaluators)
        add_executable(test_${t} unit/test_${t}.cpp)
        target_link_libraries(test_${t} PRIVATE simpleslam_eval Catch2::Catch2WithMain)
        catch_discover_tests(test_${t})
    endforeach()

    add_executable(test_framework_calibration unit/test_framework_calibration.cpp)
    target_link_libraries(test_framework_calibration
        PRIVATE simpleslam_config simpleslam_modules Catch2::Catch2WithMain)
    catch_discover_tests(test_framework_calibration)

    add_executable(test_sources_kitti unit/test_sources_kitti.cpp)
    target_link_libraries(test_sources_kitti PRIVATE simpleslam_plugins Catch2::Catch2WithMain)
    catch_discover_tests(test_sources_kitti)

    add_executable(test_config_schema unit/test_config_schema.cpp)
    target_link_libraries(test_config_schema PRIVATE simpleslam_config Catch2::Catch2WithMain)
    catch_discover_tests(test_config_schema)

    add_executable(test_system_evaluation unit/test_system_evaluation.cpp)
    target_link_libraries(test_system_evaluation
        PRIVATE simpleslam_app simpleslam_eval Catch2::Catch2WithMain)
    catch_discover_tests(test_system_evaluation)

    # 验收场景：按"系统在这种情形下必须怎样"组织，跨通信、打包、标定、评测
    add_executable(test_acceptance unit/test_acceptance.cpp)
    target_link_libraries(test_acceptance
        PRIVATE simpleslam_modules simpleslam_eval simpleslam_config
                Catch2::Catch2WithMain)
    catch_discover_tests(test_acceptance)

    # 相邻的状态转换：两件事紧挨着发生时，终态证据、寿命与结论是否仍然成立
    add_executable(test_transitions unit/test_transitions.cpp)
    target_link_libraries(test_transitions
        PRIVATE simpleslam_app simpleslam_eval Catch2::Catch2WithMain)
    catch_discover_tests(test_transitions)

    # 任务执行器：从任务文件到任务记录。多序列的子进程要登记同一个测试实现，另有一个入口
    add_executable(test_task_runner unit/task_runner_main.cpp)
    target_link_libraries(test_task_runner PRIVATE simpleslam_app)
    add_executable(test_app_task unit/test_app_task.cpp)
    target_link_libraries(test_app_task PRIVATE simpleslam_app Catch2::Catch2WithMain)
    target_compile_definitions(test_app_task PRIVATE
        SIMPLESLAM_TEST_TASK_RUNNER="$<TARGET_FILE:test_task_runner>")
    add_dependencies(test_app_task test_task_runner)
    catch_discover_tests(test_app_task)

    # 机制验收：用户的实现经契约模板一行验收；只改 YAML，方案就换实现
    add_executable(test_mechanism_acceptance unit/test_mechanism_acceptance.cpp)
    target_link_libraries(test_mechanism_acceptance
        PRIVATE simpleslam_app simpleslam_testing Catch2::Catch2WithMain)
    catch_discover_tests(test_mechanism_acceptance)

    add_executable(test_contract_boundaries unit/test_contract_boundaries.cpp)
    target_link_libraries(test_contract_boundaries
        PRIVATE simpleslam_modules Catch2::Catch2WithMain)
    catch_discover_tests(test_contract_boundaries PROPERTIES TIMEOUT 30)

    # Python 侧（配置渲染、轨迹、指标），外加与 C++ 的逐字节 / 逐数字比对。
    # 没有 python3 就不登记：跳过要可见，不能假装通过。
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME python_suite
                 COMMAND ${Python3_EXECUTABLE} -m unittest discover -s
                         "${PROJECT_SOURCE_DIR}/python/tests"
                 WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
        set_tests_properties(python_suite PROPERTIES ENVIRONMENT
            "PYTHONPATH=${PROJECT_SOURCE_DIR}/python;SIMPLESLAM_BUILD_DIR=${PROJECT_BINARY_DIR}")
    else()
        message(STATUS "未找到 python3：Python 侧测试与 C++/Python 比对不登记")
    endif()
endif()

# 模板与内联代码在头文件里，由包含它们的测试翻译单元编译。静态库 PRIVATE 链接的
# 编译选项不会传播到这些翻译单元；只在链接时带 sanitizer 检测不到这部分代码。
# 每个测试目标自己 PRIVATE 使用选项，不把工程内的警告规则传给库的消费者。
get_property(simpleslam_new_test_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(test_target IN LISTS simpleslam_new_test_targets)
    get_target_property(test_target_type ${test_target} TYPE)
    if(test_target_type STREQUAL "EXECUTABLE")
        target_link_libraries(${test_target} PRIVATE simpleslam_compiler_options)
    endif()
endforeach()
