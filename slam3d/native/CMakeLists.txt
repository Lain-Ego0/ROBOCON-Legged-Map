cmake_minimum_required(VERSION 3.22)
project(robocon_glim_native LANGUAGES CXX)

if(NOT DEFINED GLIM_SOURCE_DIR)
  message(FATAL_ERROR "Set GLIM_SOURCE_DIR to the patched GLIM source tree")
endif()

set(BUILD_WITH_CUDA OFF CACHE BOOL "Build GLIM CUDA modules" FORCE)
set(BUILD_WITH_VIEWER OFF CACHE BOOL "Disable GLIM viewer" FORCE)
set(BUILD_WITH_OPENCV OFF CACHE BOOL "Disable optional OpenCV" FORCE)
set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Keep portable CPU code" FORCE)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_subdirectory("${GLIM_SOURCE_DIR}" glim)

add_executable(mujoco_glim_bridge glim_bridge.cpp)
target_link_libraries(mujoco_glim_bridge PRIVATE glim)
target_compile_features(mujoco_glim_bridge PRIVATE cxx_std_17)
add_dependencies(mujoco_glim_bridge odometry_estimation_cpu sub_mapping_passthrough global_mapping_pose_graph)
set_target_properties(mujoco_glim_bridge PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
