cmake_minimum_required(VERSION 3.10)
project(ethercat_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
    /home/roberto-grajales/SOEM/include
    /home/roberto-grajales/SOEM/build/include
    /home/roberto-grajales/SOEM/osal
    /home/roberto-grajales/SOEM/osal/linux
    /home/roberto-grajales/SOEM/oshw/linux
)

add_executable(ethercat_publisher_node src/ethercat_publisher_node.cpp)

target_link_libraries(ethercat_publisher_node
    /home/roberto-grajales/SOEM/build/libsoem.a
    pthread
)

ament_target_dependencies(ethercat_publisher_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
)

install(TARGETS ethercat_publisher_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
