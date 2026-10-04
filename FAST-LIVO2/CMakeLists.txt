cmake_minimum_required(VERSION 3.10)
project(fast_livo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|AARCH64)$")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # Keep Eigen's ABI compatible with the distribution PCL binaries. Enabling
  # AVX/AVX-512 through -march=native changes Eigen's aligned allocator ABI
  # across the shared-library boundary and can crash while freeing PCL clouds.
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -funroll-loops") #-flto
  add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16
                  -DEIGEN_MALLOC_ALREADY_ALIGNED=1)
  message(STATUS "Using ABI-safe x86 optimizations: -O3 -march=x86-64 -mtune=generic -funroll-loops; Eigen malloc ABI: 16-byte system alignment")
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

# vio.cpp queries the LiDAR voxel map directly; make that shared-library
# dependency explicit instead of relying on the final executable's link order.
target_link_libraries(vio lio)

if(OpenMP_CXX_FOUND)
  foreach(openmp_target vio lio pre imu_proc laser_mapping)
    target_link_libraries(${openmp_target} OpenMP::OpenMP_CXX)
  endforeach()
endif()

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

# Finite-value guards are part of the estimator's failure-containment path.
# Keep IEEE NaN/Inf semantics explicit on every core translation unit.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(core_target vio lio pre imu_proc laser_mapping fastlivo_mapping)
    target_compile_options(${core_target} PRIVATE -fno-fast-math)
  endforeach()
endif()

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(recovery_supervisor_test test/recovery_supervisor_test.cpp)
  catkin_add_gtest(recovery_scan_matcher_test test/recovery_scan_matcher_test.cpp)
  if(TARGET recovery_scan_matcher_test)
    target_link_libraries(recovery_scan_matcher_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(recovery_scan_matcher_test OpenMP::OpenMP_CXX)
    endif()
  endif()
  catkin_add_gtest(so3_covariance_reset_test test/so3_covariance_reset_test.cpp)
  if(TARGET so3_covariance_reset_test AND OpenMP_CXX_FOUND)
    target_link_libraries(so3_covariance_reset_test OpenMP::OpenMP_CXX)
  endif()
  catkin_add_gtest(voxel_map_recovery_test test/voxel_map_recovery_test.cpp)
  if(TARGET voxel_map_recovery_test)
    target_link_libraries(voxel_map_recovery_test lio ${catkin_LIBRARIES} ${PCL_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(voxel_map_recovery_test OpenMP::OpenMP_CXX)
    endif()
  endif()
  catkin_add_gtest(vio_warp_test test/vio_warp_test.cpp)
  if(TARGET vio_warp_test)
    # The test itself does not call ROS, but liblio does. Keep those indirect
    # shared-library dependencies from being discarded by GNU ld --as-needed.
    target_link_libraries(vio_warp_test vio lio -Wl,--no-as-needed
                          ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(vio_warp_test OpenMP::OpenMP_CXX)
    endif()
  endif()
endif()
