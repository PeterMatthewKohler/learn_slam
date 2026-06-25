#include "slam_ros_node/SlamRosNode_impl.hpp"
#include <cstdio>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto pSlamNode = std::make_shared<slam_ros_node::SlamRosNode>(options);
    executor.add_node(pSlamNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


