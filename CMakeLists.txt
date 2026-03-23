cmake_minimum_required(VERSION 3.20)

project(SimpleSLAM VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Sophus QUIET)

if(TARGET spdlog::spdlog)
    set(SIMPLESLAM_SPDLOG_TARGET spdlog::spdlog)
elseif(TARGET spdlog::spdlog_header_only)
    set(SIMPLESLAM_SPDLOG_TARGET spdlog::spdlog_header_only)
else()
    message(FATAL_ERROR "spdlog was found but no supported imported target is available")
endif()

if(NOT TARGET Sophus::Sophus)
    if(Sophus_FOUND)
        if(DEFINED Sophus_INCLUDE_DIRS)
            set(SIMPLESLAM_SOPHUS_INCLUDE_DIRS "${Sophus_INCLUDE_DIRS}")
        elseif(DEFINED Sophus_INCLUDE_DIR)
            set(SIMPLESLAM_SOPHUS_INCLUDE_DIRS "${Sophus_INCLUDE_DIR}")
        else()
            message(FATAL_ERROR "Sophus was found but no include directory was exported")
        endif()
    else()
        set(_simpleslam_sophus_hints)
        if(DEFINED SIMPLESLAM_SOPHUS_INCLUDE_DIR)
            list(APPEND _simpleslam_sophus_hints "${SIMPLESLAM_SOPHUS_INCLUDE_DIR}")
        endif()
        list(APPEND _simpleslam_sophus_hints
            "${PROJECT_SOURCE_DIR}/third_party"
            "${PROJECT_SOURCE_DIR}/third_party/Sophus"
            "${PROJECT_SOURCE_DIR}/external"
            "${PROJECT_SOURCE_DIR}/external/Sophus"
        )

        find_path(SIMPLESLAM_SOPHUS_INCLUDE_DIR
            NAMES sophus/se3.hpp
            HINTS ${_simpleslam_sophus_hints}
            PATH_SUFFIXES include
        )
        if(NOT SIMPLESLAM_SOPHUS_INCLUDE_DIR)
            message(FATAL_ERROR "Sophus not found. Install Sophus, set Sophus_DIR, or pass -DSIMPLESLAM_SOPHUS_INCLUDE_DIR=/path/to/include.")
        endif()
        set(SIMPLESLAM_SOPHUS_INCLUDE_DIRS "${SIMPLESLAM_SOPHUS_INCLUDE_DIR}")
    endif()

    add_library(Sophus::Sophus INTERFACE IMPORTED)
    target_include_directories(Sophus::Sophus INTERFACE ${SIMPLESLAM_SOPHUS_INCLUDE_DIRS})
endif()

file(GLOB_RECURSE SIMPLESLAM_SOURCES CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

if(NOT SIMPLESLAM_SOURCES)
    message(FATAL_ERROR "No source files were found under ${PROJECT_SOURCE_DIR}/src")
endif()

add_library(simpleslam ${SIMPLESLAM_SOURCES})
add_library(SimpleSLAM::simpleslam ALIAS simpleslam)

target_compile_features(simpleslam PUBLIC cxx_std_20)

# GCC < 10 needs -fconcepts for C++20 requires-clauses
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10")
    target_compile_options(simpleslam PUBLIC -fconcepts)
endif()

target_include_directories(simpleslam
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(simpleslam
    PUBLIC
        Eigen3::Eigen
        Sophus::Sophus
        ${SIMPLESLAM_SPDLOG_TARGET}
        yaml-cpp
)
