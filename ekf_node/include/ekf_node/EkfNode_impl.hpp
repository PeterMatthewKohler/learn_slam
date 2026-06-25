#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace ekf_node {
    class EkfNode : public rclcpp::Node
    {
    public:
        explicit EkfNode(const rclcpp::NodeOptions &options);
    
    private:
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekfPublisher_;
        // Subscribers
        rclcpp::Subscriber<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
        
    }
}