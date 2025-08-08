#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <optional>
#include <random>

namespace noisy_odom {
    class NoisyOdomNode : public rclcpp::Node
    {
        public:
        NoisyOdomNode(const rclcpp::NodeOptions& options);

        private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisyOdomPublisher;
        void publishNoisyOdom();
        // Timer
        rclcpp::TimerBase::SharedPtr noisyOdomTimer;
        // Internal states
        std::optional<nav_msgs::msg::Odometry::SharedPtr> currentVehOdom;
        nav_msgs::msg::Odometry::SharedPtr noisyOdom;
        // Mutex for thread safety
        std::mutex mutex_;
        // Parameters
        double pos_stddev_;
        double rot_stddev_;
        // Random number generator
        std::mt19937 gen_;
        std::normal_distribution<> pos_dist_;
        std::normal_distribution<> rot_dist_;
    };
}