# 录制与回放层：容器格式、录制器、回放来源、实验回放包、诊断落盘。
add_library(simpleslam_record STATIC
    src/format.cpp
    src/writer.cpp
    src/reader.cpp
    src/bundle.cpp
    src/diagnostic_file_sink.cpp
)
target_include_directories(simpleslam_record PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_features(simpleslam_record PUBLIC cxx_std_20)
target_link_libraries(simpleslam_record
    PUBLIC simpleslam_framework
    PRIVATE ${SIMPLESLAM_YAMLCPP_TARGET}
)
if(TARGET simpleslam_compiler_options)
    target_link_libraries(simpleslam_record PRIVATE simpleslam_compiler_options)
endif()
add_library(SimpleSLAM::record ALIAS simpleslam_record)
