# SimpleSLAM 编译选项模块
# 定义 simpleslam_compiler_options INTERFACE 库，统一管理编译标志

add_library(simpleslam_compiler_options INTERFACE)

# ── 警告标志 ──
target_compile_options(simpleslam_compiler_options INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter      # 接口类中未使用的参数很常见
)

# ── 可选：AddressSanitizer / ThreadSanitizer / UBSan ──
set(SIMPLESLAM_SANITIZER "" CACHE STRING "启用 sanitizer（address / thread / undefined）")
if(SIMPLESLAM_SANITIZER)
    message(STATUS "SimpleSLAM: 启用 sanitizer = ${SIMPLESLAM_SANITIZER}")
    target_compile_options(simpleslam_compiler_options INTERFACE
        -fsanitize=${SIMPLESLAM_SANITIZER}
        -fno-omit-frame-pointer
    )
    target_link_options(simpleslam_compiler_options INTERFACE
        -fsanitize=${SIMPLESLAM_SANITIZER}
    )
endif()

# ── 可选：LTO（链接时优化）──
option(SIMPLESLAM_ENABLE_LTO "启用链接时优化" OFF)
if(SIMPLESLAM_ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT lto_supported)
    if(lto_supported)
        message(STATUS "SimpleSLAM: 启用 LTO")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    else()
        message(WARNING "SimpleSLAM: 编译器不支持 LTO，已忽略")
    endif()
endif()
