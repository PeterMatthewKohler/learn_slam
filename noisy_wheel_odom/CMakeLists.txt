cmake_minimum_required(VERSION 3.8)
project(noisy_wheel_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

set(NoisyWheelOdomNode_SRC
  src/noisy_wheel_odom/NoisyWheelOdomNode_impl.cpp
)
set(NoisyWheelOdomNode_HEADERS
  include/noisy_wheel_odom/NoisyWheelOdomNode_impl.hpp
)

ament_auto_add_library(NoisyWheelOdomNode_lib
  ${NoisyWheelOdomNode_SRC}
  ${NoisyWheelOdomNode_HEADERS}
)

ament_target_dependencies(NoisyWheelOdomNode_lib
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
)

# Link non-ament packages explicitly
target_link_libraries(NoisyWheelOdomNode_lib Eigen3::Eigen)

ament_auto_add_executable(NoisyWheelOdomNode_exe
  src/noisy_wheel_odom/NoisyWheelOdomNode_main.cpp
)

target_link_libraries(NoisyWheelOdomNode_exe
  NoisyWheelOdomNode_lib
)

install(TARGETS NoisyWheelOdomNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install parameter files
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


