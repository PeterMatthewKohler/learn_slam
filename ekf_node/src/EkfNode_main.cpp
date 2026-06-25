#include "efk_node/EkfNode_impl.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, **argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    auto node = std::make_shared<noisy_imu::NoisyImuNode>(options);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}