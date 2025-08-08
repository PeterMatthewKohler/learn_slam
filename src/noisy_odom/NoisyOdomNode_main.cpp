#include "noisy_odom/NoisyOdomNode_impl.hpp"
#include <cstdio>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto pNoisyOdomNode = std::make_shared<noisy_odom::NoisyOdomNode>(options);
    executor.add_node(pNoisyOdomNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}