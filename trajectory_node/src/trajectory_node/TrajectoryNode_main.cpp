#include "trajectory_node/TrajectoryNode_impl.hpp"
#include <cstdio>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto pTrajectoryNode = std::make_shared<trajectory_node::TrajectoryNode>(options);
    executor.add_node(pTrajectoryNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


