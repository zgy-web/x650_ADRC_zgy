cmake_minimum_required(VERSION 3.16)
project(x650_memory_benchmark LANGUAGES CXX)

# Standalone, Linux-only CPU harness. The production mathematics is built from
# its existing sources; ROS, Python and a simulation executable are not needed.
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
include(CTest)
add_subdirectory(../native native)
add_library(x650_benchmark_predictor SHARED ../x650_wholebody/ts20_prediction_native.cpp)
target_compile_features(x650_benchmark_predictor PRIVATE cxx_std_17)
target_compile_options(x650_benchmark_predictor PRIVATE -Wall -Wextra)
add_executable(x650_mpc_memory_benchmark src/memory_benchmark.cpp)
target_include_directories(x650_mpc_memory_benchmark PRIVATE include ../native)
target_compile_features(x650_mpc_memory_benchmark PRIVATE cxx_std_17)
target_compile_options(x650_mpc_memory_benchmark PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(x650_mpc_memory_benchmark PRIVATE x650_mpc_core
  PkgConfig::JSONCPP OpenSSL::Crypto Threads::Threads ${CMAKE_DL_LIBS})
set_target_properties(x650_mpc_memory_benchmark PROPERTIES
  BUILD_RPATH "$ORIGIN;$ORIGIN/native;$ORIGIN/lib"
  INSTALL_RPATH "$ORIGIN;$ORIGIN/lib")

if(BUILD_TESTING)
  add_executable(test_benchmark_io test/test_benchmark_io.cpp)
  target_include_directories(test_benchmark_io PRIVATE include ../native)
  target_compile_features(test_benchmark_io PRIVATE cxx_std_17)
  target_link_libraries(test_benchmark_io PRIVATE x650_mpc_core
    PkgConfig::JSONCPP OpenSSL::Crypto Threads::Threads ${CMAKE_DL_LIBS})
  add_test(NAME memory_benchmark_input_contract COMMAND test_benchmark_io)
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME memory_benchmark_process_contract COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_process.py
      --binary $<TARGET_FILE:x650_mpc_memory_benchmark>
      --predictor $<TARGET_FILE:x650_benchmark_predictor>)
  endif()
endif()
