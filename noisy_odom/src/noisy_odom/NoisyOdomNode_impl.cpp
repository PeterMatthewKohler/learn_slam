#include "noisy_odom/NoisyOdomNode_impl.hpp"

namespace noisy_odom {
    NoisyOdomNode::NoisyOdomNode(const rclcpp::NodeOptions& options) : Node("noisy_odom_node", options)
    {
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/nova/odom", 10, std::bind(&NoisyOdomNode::odomCallback, this, std::placeholders::_1));
        noisyOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(
            "/nova/noisy_odom", 10);
        noisyOdomTimer = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&NoisyOdomNode::publishNoisyOdom, this));

        this->declare_parameter("pos_noise_stddev", 0.01);
        this->declare_parameter("rot_noise_stddev", 0.01);
        pos_stddev_ = this->get_parameter("pos_noise_stddev").as_double();
        rot_stddev_ = this->get_parameter("rot_noise_stddev").as_double();

        std::random_device rd;
        gen_ = std::mt19937(rd());
        pos_dist_ = std::normal_distribution<>(0.0, pos_stddev_);
        rot_dist_ = std::normal_distribution<>(0.0, rot_stddev_);
    }

    void NoisyOdomNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentVehOdom = msg;
    }

    void NoisyOdomNode::publishNoisyOdom()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!currentVehOdom.has_value())
        {
            return;
        }
        const nav_msgs::msg::Odometry::SharedPtr& odom = currentVehOdom.value();
        noisyOdom = std::make_shared<nav_msgs::msg::Odometry>(*odom);
        noisyOdom->pose.pose.position.x += pos_dist_(gen_);
        noisyOdom->pose.pose.position.y += pos_dist_(gen_);
        noisyOdom->pose.pose.position.z += pos_dist_(gen_);
        noisyOdom->pose.pose.orientation.x += rot_dist_(gen_);
        noisyOdom->pose.pose.orientation.y += rot_dist_(gen_);
        noisyOdom->pose.pose.orientation.z += rot_dist_(gen_);
        noisyOdom->pose.pose.orientation.w += rot_dist_(gen_);
        double norm = std::sqrt(
            noisyOdom->pose.pose.orientation.x * noisyOdom->pose.pose.orientation.x +
            noisyOdom->pose.pose.orientation.y * noisyOdom->pose.pose.orientation.y +
            noisyOdom->pose.pose.orientation.z * noisyOdom->pose.pose.orientation.z +
            noisyOdom->pose.pose.orientation.w * noisyOdom->pose.pose.orientation.w);
        noisyOdom->pose.pose.orientation.x /= norm;
        noisyOdom->pose.pose.orientation.y /= norm;
        noisyOdom->pose.pose.orientation.z /= norm;
        noisyOdom->pose.pose.orientation.w /= norm;
        noisyOdom->pose.covariance = odom->pose.covariance;
        for (int i = 0; i < 6; ++i) {
            noisyOdom->pose.covariance[i * 6 + i] +=
                (i < 3 ? pos_stddev_ * pos_stddev_ : rot_stddev_ * rot_stddev_);
        }
        noisyOdom->twist.twist.linear.x += pos_dist_(gen_);
        noisyOdom->twist.twist.linear.y += pos_dist_(gen_);
        noisyOdom->twist.twist.linear.z += pos_dist_(gen_);
        noisyOdom->twist.twist.angular.x += rot_dist_(gen_);
        noisyOdom->twist.twist.angular.y += rot_dist_(gen_);
        noisyOdom->twist.twist.angular.z += rot_dist_(gen_);
        noisyOdom->twist.covariance = odom->twist.covariance;
        for (int i = 0; i < 6; ++i) {
            noisyOdom->twist.covariance[i * 6 + i] +=
                (i < 3 ? pos_stddev_ * pos_stddev_ : rot_stddev_ * rot_stddev_);
        }
        noisyOdomPublisher->publish(*noisyOdom);
    }
}


