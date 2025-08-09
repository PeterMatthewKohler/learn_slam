cmake_minimum_required(VERSION 3.8)
project(noisy_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)

set(NoisyOdomNode_SRC
  src/noisy_odom/NoisyOdomNode_impl.cpp
)
set(NoisyOdomNode_HEADERS
  include/noisy_odom/NoisyOdomNode_impl.hpp
)

ament_auto_add_library(NoisyOdomNode_lib
  ${NoisyOdomNode_SRC}
  ${NoisyOdomNode_HEADERS}
)

ament_target_dependencies(NoisyOdomNode_lib
  rclcpp
  rclcpp_components
  nav_msgs
)

ament_auto_add_executable(NoisyOdomNode_exe
  src/noisy_odom/NoisyOdomNode_main.cpp
)

target_link_libraries(NoisyOdomNode_exe
  NoisyOdomNode_lib
)

install(TARGETS NoisyOdomNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


