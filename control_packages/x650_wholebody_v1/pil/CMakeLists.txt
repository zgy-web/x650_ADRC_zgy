cmake_minimum_required(VERSION 3.16)
project(x650_pil LANGUAGES CXX)
# Link the already-qualified ABI 4 binaries; this adapter does not rebuild math.
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
find_package(OpenSSL REQUIRED)
if(NOT IS_ABSOLUTE "${X650_RUNTIME}" OR NOT EXISTS "${X650_RUNTIME}/lib/libx650_mpc_core.so")
  message(FATAL_ERROR "Explicit existing ABI 4 runtime required")
endif()
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
add_executable(x650_mpc_pil_worker worker.cpp)
target_compile_features(x650_mpc_pil_worker PRIVATE cxx_std_17)
target_compile_options(x650_mpc_pil_worker PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(x650_mpc_pil_worker PRIVATE ../benchmark/include ../native
  "${REPO}/ros2_ws/src/flyarm_mpc_cpp/include"
  "${REPO}/ros2_ws/src/flyarm_state_cpp/include"
  "${REPO}/ros2_ws/src/flyarm_control_gateway/include")
target_link_libraries(x650_mpc_pil_worker PRIVATE "${X650_RUNTIME}/lib/libx650_mpc_core.so"
  "${X650_RUNTIME}/lib/libx650_optimization.so" PkgConfig::JSONCPP OpenSSL::Crypto ${CMAKE_DL_LIBS})
set_target_properties(x650_mpc_pil_worker PROPERTIES BUILD_RPATH "${X650_RUNTIME}/lib")
include(CTest)
if(BUILD_TESTING)
  add_executable(test_pil_reference test_reference.cpp)
  target_compile_features(test_pil_reference PRIVATE cxx_std_17)
  target_include_directories(test_pil_reference PRIVATE ../native
    "${REPO}/ros2_ws/src/flyarm_mpc_cpp/include"
    "${REPO}/ros2_ws/src/flyarm_state_cpp/include"
    "${REPO}/ros2_ws/src/flyarm_control_gateway/include")
  add_test(NAME cross_arch_reference_roundoff COMMAND test_pil_reference)
endif()
