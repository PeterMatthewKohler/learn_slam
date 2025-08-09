cmake_minimum_required(VERSION 3.8)
project(trajectory_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

set(TrajectoryNode_SRC
  src/trajectory_node/TrajectoryNode_impl.cpp
)
set(TrajectoryNode_HEADERS
  include/trajectory_node/TrajectoryNode_impl.hpp
)

ament_auto_add_library(TrajectoryNode_lib
  ${TrajectoryNode_SRC}
  ${TrajectoryNode_HEADERS}
)

ament_target_dependencies(TrajectoryNode_lib
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
)

ament_auto_add_executable(TrajectoryNode_exe
  src/trajectory_node/TrajectoryNode_main.cpp
)

target_link_libraries(TrajectoryNode_exe
  TrajectoryNode_lib
  nlohmann_json::nlohmann_json
)

install(TARGETS TrajectoryNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(FILES example_trajectory.json
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


