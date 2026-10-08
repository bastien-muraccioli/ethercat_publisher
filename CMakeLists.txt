cmake_minimum_required(VERSION 3.14)
project(ethercat_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ── SOEM (>= 2.0) ─────────────────────────────────────────────────────────────
option(ETHERCAT_PUBLISHER_FETCH_SOEM "Download and build SOEM if it is not found" ON)
set(SOEM_GIT_TAG "v2.0.0" CACHE STRING "SOEM version to fetch")

set(_soem_ok FALSE)
find_package(soem CONFIG QUIET)
if(soem_FOUND AND TARGET soem)
  get_target_property(_soem_inc soem INTERFACE_INCLUDE_DIRECTORIES)
  find_file(_soem_header soem/soem.h PATHS ${_soem_inc} NO_DEFAULT_PATH NO_CACHE)
  if(_soem_header)
    set(_soem_ok TRUE)
    message(STATUS "Using installed SOEM: ${soem_DIR}")
  else()
    message(STATUS "Ignoring SOEM at ${soem_DIR}: not SOEM >= 2.0 (no soem/soem.h)")
  endif()
endif()

if(NOT _soem_ok)
  if(NOT ETHERCAT_PUBLISHER_FETCH_SOEM)
    message(FATAL_ERROR "SOEM >= 2.0 not found and ETHERCAT_PUBLISHER_FETCH_SOEM is OFF")
  endif()
  message(STATUS "Fetching SOEM ${SOEM_GIT_TAG}")
  include(FetchContent)
  FetchContent_Declare(soem_src
    GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM.git
    GIT_TAG        ${SOEM_GIT_TAG}
    GIT_SHALLOW    TRUE
    EXCLUDE_FROM_ALL
  )
  FetchContent_MakeAvailable(soem_src)
endif()

if(NOT TARGET soem)
  message(FATAL_ERROR "SOEM target 'soem' is not available")
endif()

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(ethercat_publisher_node src/ethercat_publisher.cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(ethercat_publisher_node PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()

target_include_directories(ethercat_publisher_node PRIVATE include)

ament_target_dependencies(ethercat_publisher_node
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs)

target_link_libraries(ethercat_publisher_node soem)   # brings includes + pthread/rt

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ethercat_publisher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()