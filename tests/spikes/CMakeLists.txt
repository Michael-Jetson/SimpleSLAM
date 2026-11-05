cmake_minimum_required(VERSION 3.20)
project(SimpleSLAMCommunicationSpike LANGUAGES CXX)

find_package(Threads REQUIRED)
option(SPIKE_STANDALONE_ASIO "Use standalone Asio instead of installed Boost.Asio" OFF)

add_executable(communication_asio communication_asio.cpp)
target_compile_features(communication_asio PRIVATE cxx_std_20)
set_target_properties(communication_asio PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(communication_asio PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(communication_asio PRIVATE Threads::Threads)

if(SPIKE_STANDALONE_ASIO)
    find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)
    target_include_directories(communication_asio PRIVATE ${ASIO_INCLUDE_DIR})
    target_compile_definitions(communication_asio PRIVATE
        ASIO_STANDALONE SIMPLESLAM_SPIKE_STANDALONE_ASIO)
else()
    find_package(Boost 1.74 CONFIG REQUIRED)
    target_link_libraries(communication_asio PRIVATE Boost::headers)
endif()

enable_testing()
add_test(NAME communication_asio COMMAND communication_asio)
set_tests_properties(communication_asio PROPERTIES TIMEOUT 15)
