#include "slam_ros_node/SlamRosNode_impl.hpp"

namespace slam_ros_node
{
    SlamRosNode::SlamRosNode (const rclcpp::NodeOptions& options) : Node("slam_ros_node", options)
    {
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/nova/noisy_odom", 10, std::bind(&SlamRosNode::odomCallback, this, std::placeholders::_1));
        pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10, std::bind(&SlamRosNode::pointCloudCallback, this, std::placeholders::_1));

        slamTimer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SlamRosNode::slamTimerCallback, this));

        slamOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/slam/odom", 10);
    }

    void SlamRosNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentVehOdom = msg;
    }

    void SlamRosNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentPointCloud = msg;
    }

    void SlamRosNode::slamTimerCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (currentVehOdom.has_value() && currentPointCloud.has_value())
        {
            slam();
        }
    }

    void SlamRosNode::slam()
    {
        // Placeholder for SLAM implementation
    }
}


