#include "slam_ros_node/SlamRosNode_impl.hpp"

namespace slam_ros_node
{
    SlamRosNode::SlamRosNode (const rclcpp::NodeOptions& options) : Node("slam_ros_node", options)
    {
        // Create subscribers
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/nova/noisy_odom", 10, std::bind(&SlamRosNode::odomCallback, this, std::placeholders::_1));
        pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10, std::bind(&SlamRosNode::pointCloudCallback, this, std::placeholders::_1));

        // Create slam timer
        slamTimer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SlamRosNode::slamTimerCallback, this));

        // Create slam output publisher
        slamOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/slam/odom", 10);
    }

    // Odometry Subscriber callback
    void SlamRosNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentVehOdom = msg;
    }
    // Point Cloud Subscriber callback
    void SlamRosNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentPointCloud = msg;
    }
    // Slam timer callback
    void SlamRosNode::slamTimerCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (currentVehOdom.has_value() && currentPointCloud.has_value()) // Only run the algorithm if we've received odometry and rangefinder updates
        {
            slam();
        }
    }

    // Slam algorithm
    void SlamRosNode::slam()
    {
        // Do nothing for now
    }
}