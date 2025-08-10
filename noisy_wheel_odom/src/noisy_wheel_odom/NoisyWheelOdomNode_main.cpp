#include "noisy_wheel_odom/NoisyWheelOdomNode_impl.hpp"
#include <cstdio>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto pNoisyWheelOdomNode = std::make_shared<noisy_wheel_odom::NoisyWheelOdomNode>(options);
    executor.add_node(pNoisyWheelOdomNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


