cmake_minimum_required(VERSION 3.8)
project(slam_ros_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(SlamRosNode_SRC
  src/slam_ros_node/SlamRosNode_impl.cpp
)
set(SlamRosNode_HEADERS
  include/slam_ros_node/SlamRosNode_impl.hpp
)

ament_auto_add_library(SlamRosNode_lib
  ${SlamRosNode_SRC}
  ${SlamRosNode_HEADERS}
)

ament_target_dependencies(SlamRosNode_lib
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
)

ament_auto_add_executable(SlamRosNode_exe
  src/slam_ros_node/SlamRosNode_main.cpp
)

target_link_libraries(SlamRosNode_exe
  SlamRosNode_lib
)

install(TARGETS SlamRosNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


