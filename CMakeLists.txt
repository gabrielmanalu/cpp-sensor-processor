cmake_minimum_required(VERSION 3.16)
project(cpp-sensor-processor VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

# Compiler flags
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Interface library
add_library(sensor_processor_lib INTERFACE)
target_include_directories(sensor_processor_lib INTERFACE include)
target_link_libraries(sensor_processor_lib INTERFACE Threads::Threads)

# Main executable
add_executable(sensor_proc src/main.cpp)
target_link_libraries(sensor_proc PRIVATE sensor_processor_lib)

# ========================
# GoogleTest (stable!)
# ========================
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

if(MSVC)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()

FetchContent_MakeAvailable(googletest)

# ========================
# Tests
# ========================
enable_testing()

add_executable(unit_tests tests/unit_tests.cpp)

target_link_libraries(unit_tests
  PRIVATE
  gtest
  gtest_main
  sensor_processor_lib
)

include(GoogleTest)
gtest_discover_tests(unit_tests)