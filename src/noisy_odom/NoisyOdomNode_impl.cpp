#include "noisy_odom/NoisyOdomNode_impl.hpp"

namespace noisy_odom {
    NoisyOdomNode::NoisyOdomNode(const rclcpp::NodeOptions& options) : Node("noisy_odom_node", options)
    {
        // Create subscribers
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/nova/odom", 10, std::bind(&NoisyOdomNode::odomCallback, this, std::placeholders::_1));
        // Create publishers
        noisyOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(
            "/nova/noisy_odom", 10);
        // Create a timer to publish the noisy odometry
        noisyOdomTimer = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&NoisyOdomNode::publishNoisyOdom, this));

        // Parameters: noise stddev
        this->declare_parameter("pos_noise_stddev", 0.01);
        this->declare_parameter("rot_noise_stddev", 0.01);
        // Get parameters
        pos_stddev_ = this->get_parameter("pos_noise_stddev").as_double();
        rot_stddev_ = this->get_parameter("rot_noise_stddev").as_double();

        // Initialize random number generator
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
        // Add white noise to the odometry
        noisyOdom = std::make_shared<nav_msgs::msg::Odometry>(*odom);
        // Add noise to the position
        noisyOdom->pose.pose.position.x += pos_dist_(gen_);
        noisyOdom->pose.pose.position.y += pos_dist_(gen_);
        noisyOdom->pose.pose.position.z += pos_dist_(gen_);
        // Add noise to the orientation
        noisyOdom->pose.pose.orientation.x += rot_dist_(gen_);
        noisyOdom->pose.pose.orientation.y += rot_dist_(gen_);
        noisyOdom->pose.pose.orientation.z += rot_dist_(gen_);
        noisyOdom->pose.pose.orientation.w += rot_dist_(gen_);
        // Normalize the quaternion
        double norm = std::sqrt(
            noisyOdom->pose.pose.orientation.x * noisyOdom->pose.pose.orientation.x +
            noisyOdom->pose.pose.orientation.y * noisyOdom->pose.pose.orientation.y +
            noisyOdom->pose.pose.orientation.z * noisyOdom->pose.pose.orientation.z +
            noisyOdom->pose.pose.orientation.w * noisyOdom->pose.pose.orientation.w);
        noisyOdom->pose.pose.orientation.x /= norm;
        noisyOdom->pose.pose.orientation.y /= norm;
        noisyOdom->pose.pose.orientation.z /= norm;
        noisyOdom->pose.pose.orientation.w /= norm;
        // Update the pose covariance matrix
        noisyOdom->pose.covariance = odom->pose.covariance;
        // Pose covariance: [x, y, z, rot_x, rot_y, rot_z]
        for (int i = 0; i < 6; ++i) {
            noisyOdom->pose.covariance[i * 6 + i] +=
                (i < 3 ? pos_stddev_ * pos_stddev_ : rot_stddev_ * rot_stddev_);
        }
        // Update the twist
        noisyOdom->twist.twist.linear.x += pos_dist_(gen_);
        noisyOdom->twist.twist.linear.y += pos_dist_(gen_);
        noisyOdom->twist.twist.linear.z += pos_dist_(gen_);
        noisyOdom->twist.twist.angular.x += rot_dist_(gen_);
        noisyOdom->twist.twist.angular.y += rot_dist_(gen_);
        noisyOdom->twist.twist.angular.z += rot_dist_(gen_);
        // Update the twist covariance matrix
        noisyOdom->twist.covariance = odom->twist.covariance;
        // Twist covariance: [vx, vy, vz, wx, wy, wz]
        for (int i = 0; i < 6; ++i) {
            noisyOdom->twist.covariance[i * 6 + i] +=
                (i < 3 ? pos_stddev_ * pos_stddev_ : rot_stddev_ * rot_stddev_);
        }
        // Publish the noisy odometry
        noisyOdomPublisher->publish(*noisyOdom);
    }
}