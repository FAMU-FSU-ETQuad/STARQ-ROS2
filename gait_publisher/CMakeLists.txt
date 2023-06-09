cmake_minimum_required(VERSION 3.8)
project(gait_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
include_directories(${YAML_CPP_INCLUDE_DIR})

# executable
add_executable(gait_publisher_node src/gait_publisher_node.cpp)
ament_target_dependencies(gait_publisher_node rclcpp sensor_msgs yaml-cpp ament_index_cpp)
target_link_libraries(gait_publisher_node ${YAML_CPP_LIBRARIES})

install(TARGETS
  gait_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
