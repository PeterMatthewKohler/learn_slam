#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mutex>
#include <optional>

namespace slam_ros_node {
    class SlamRosNode : public rclcpp::Node
    {
        public:
        SlamRosNode(const rclcpp::NodeOptions& options);

        private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub;
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        // Timer
        rclcpp::TimerBase::SharedPtr slamTimer;
        void slamTimerCallback();
        // Slam algorithm
        void slam();
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr slamOdomPublisher;
        void publishSlamOdom(const nav_msgs::msg::Odometry::SharedPtr& odom);
        std::optional<nav_msgs::msg::Odometry::SharedPtr> currentVehOdom;
        std::optional<sensor_msgs::msg::PointCloud2::SharedPtr> currentPointCloud;
        nav_msgs::msg::Odometry::SharedPtr slamOdom;
        std::mutex mutex_;
    };
}


