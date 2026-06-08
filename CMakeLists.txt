cmake_minimum_required(VERSION 3.14)
project(ethercat_publisher)

# ── Require C++17 ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── SOEM ─────────────────────────────────────────────────────────────────────
# SOEM is built from source. Point cmake to it via -DSOEM_DIR=<path>
# or let it auto-detect from the default ~/SOEM location.
#
# Usage:
#   colcon build ... --cmake-args -DSOEM_DIR=/path/to/SOEM
#
# If SOEM_DIR is not passed, we try the default ~/SOEM path as a convenience
# for the primary developer. CI / other developers must pass -DSOEM_DIR.

if(NOT DEFINED SOEM_DIR)
  set(SOEM_DIR "$ENV{HOME}/SOEM")
  message(STATUS "SOEM_DIR not set — using default: ${SOEM_DIR}")
endif()

set(SOEM_INCLUDE_DIRS
  ${SOEM_DIR}/include
  ${SOEM_DIR}/build/include
  ${SOEM_DIR}/osal
  ${SOEM_DIR}/osal/linux
  ${SOEM_DIR}/oshw/linux
)

set(SOEM_LIBRARY ${SOEM_DIR}/build/libsoem.a)

if(NOT EXISTS ${SOEM_LIBRARY})
  message(FATAL_ERROR
    "SOEM static library not found at ${SOEM_LIBRARY}.\n"
    "Build SOEM first:\n"
    "  cd ~/SOEM && mkdir -p build && cd build && cmake .. && make\n"
    "Or pass the correct path:\n"
    "  colcon build --cmake-args -DSOEM_DIR=/path/to/SOEM")
endif()

message(STATUS "Using SOEM: ${SOEM_LIBRARY}")

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(ethercat_publisher_node
  src/ethercat_publisher.cpp
)

target_include_directories(ethercat_publisher_node PRIVATE
  include
  ${SOEM_INCLUDE_DIRS}
)

ament_target_dependencies(ethercat_publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

target_link_libraries(ethercat_publisher_node
  ${SOEM_LIBRARY}
  pthread
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ethercat_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
