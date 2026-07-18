#include "vio_node/VIONode.hpp"
#include <cstdio>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto pVIONode = std::make_shared<vio_node::VIONode>(options);
    executor.add_node(pVIONode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
