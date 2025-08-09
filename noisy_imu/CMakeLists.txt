cmake_minimum_required(VERSION 3.8)
project(noisy_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

set(NoisyImuNode_SRC
  src/noisy_imu/NoisyImuNode_impl.cpp
)
set(NoisyImuNode_HEADERS
  include/noisy_imu/NoisyImuNode_impl.hpp
)

ament_auto_add_library(NoisyImuNode_lib
  ${NoisyImuNode_SRC}
  ${NoisyImuNode_HEADERS}
)

ament_target_dependencies(NoisyImuNode_lib
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_auto_add_executable(NoisyImuNode_exe
  src/noisy_imu/NoisyImuNode_main.cpp
)

target_link_libraries(NoisyImuNode_exe
  NoisyImuNode_lib
)

install(TARGETS NoisyImuNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


