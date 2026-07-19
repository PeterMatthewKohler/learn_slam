cmake_minimum_required(VERSION 3.8)
project(vio_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(VIONode_SRC
    src/ErrorStateEkf.cpp
    src/EstimatorBackend.cpp
    src/EstimatorInitialization.cpp
    src/EstimatorOutput.cpp
    src/ImuProcessing.cpp
    src/Parameters.cpp
    src/VIONode.cpp
    src/StereoFrontend.cpp
    src/Transforms.cpp
    src/VisualOdometry.cpp
)

set(VIONode_HEADERS
    include/vio_node/ErrorStateEkf.hpp
    include/vio_node/VIONode.hpp
    include/vio_node/EstimatorTypes.hpp
    include/vio_node/ImuTypes.hpp
    include/vio_node/Validation.hpp
    include/vio_node/VisionTypes.hpp
)

ament_auto_add_library(VIONode_lib
    ${VIONode_SRC}
    ${VIONode_HEADERS}
)

ament_target_dependencies(VIONode_lib
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    message_filters
    tf2
    tf2_ros
    Eigen3
    OpenCV
    cv_bridge
)

ament_auto_add_executable(VIONode_exe
    src/VIONode_main.cpp
)

target_link_libraries(VIONode_exe
    VIONode_lib
)

install(TARGETS VIONode_exe
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
