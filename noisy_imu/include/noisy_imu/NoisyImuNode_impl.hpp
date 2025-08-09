#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <random>

namespace noisy_imu {
    class NoisyImuNode : public rclcpp::Node
    {
    public:
        explicit NoisyImuNode(const rclcpp::NodeOptions &options);

    private:
        // Subscriber and publisher
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr noisyImuPublisher_;

        // Callback
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        // Parameters
        double orientationStddev_{};
        double angularVelocityStddev_{};
        double linearAccelerationStddev_{};

        // RNG
        std::mt19937 randomGenerator_;
        std::normal_distribution<> orientationNoiseDist_;
        std::normal_distribution<> angularVelocityNoiseDist_;
        std::normal_distribution<> linearAccelerationNoiseDist_;
    };
}


