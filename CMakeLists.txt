cmake_minimum_required(VERSION 3.8)
project(slam_ros_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

#--------------------------------------------------------------
#                     SlamRosNode Node
#--------------------------------------------------------------
# List sources
set(SlamRosNode_SRC
  src/slam_ros_node/SlamRosNode_impl.cpp
)
# List headers
set(SlamRosNode_HEADERS
  include/slam_ros_node/SlamRosNode_impl.hpp
)
# Create a shared (.so) library
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

# Create an executable
ament_auto_add_executable(SlamRosNode_exe
    src/slam_ros_node/SlamRosNode_main.cpp
)

# Link the library to the executable
target_link_libraries(SlamRosNode_exe
  SlamRosNode_lib
)

# Install the executable
install(TARGETS SlamRosNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

#--------------------------------------------------------------
#                     NoisyOdomNode Node
#--------------------------------------------------------------
# List sources
set(NoisyOdomNode_SRC
  src/noisy_odom/NoisyOdomNode_impl.cpp
)
# List headers
set(NoisyOdomNode_HEADERS
  include/noisy_odom/NoisyOdomNode_impl.hpp
)
# Create a shared (.so) library
ament_auto_add_library(NoisyOdomNode_lib
    ${NoisyOdomNode_SRC}
    ${NoisyOdomNode_HEADERS}
)

ament_target_dependencies(NoisyOdomNode_lib
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
)

# Create an executable
ament_auto_add_executable(NoisyOdomNode_exe
    src/noisy_odom/NoisyOdomNode_main.cpp
)

# Link the library to the executable
target_link_libraries(NoisyOdomNode_exe
  NoisyOdomNode_lib
)

# Install the executable
install(TARGETS NoisyOdomNode_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
