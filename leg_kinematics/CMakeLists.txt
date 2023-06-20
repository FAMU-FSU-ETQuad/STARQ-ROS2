cmake_minimum_required(VERSION 3.8)
project(leg_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
include_directories(${YAML_CPP_INCLUDE_DIR})

# executable
add_executable(leg_kinematics_node src/leg_kinematics_node.cpp)
ament_target_dependencies(leg_kinematics_node rclcpp trajectory_msgs sensor_msgs yaml-cpp ament_index_cpp)
target_link_libraries(leg_kinematics_node ${YAML_CPP_LIBRARIES})

install(TARGETS
  leg_kinematics_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
