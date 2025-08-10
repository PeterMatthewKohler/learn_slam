#include "noisy_wheel_odom/NoisyWheelOdomNode_impl.hpp"

namespace noisy_wheel_odom {
    NoisyWheelOdomNode::NoisyWheelOdomNode(const rclcpp::NodeOptions& options) : Node("noisy_wheel_odom_node", options)
    {
        // Initialize subscribers and publishers
        jointStateSub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/nova/joint_states", 10, std::bind(&NoisyWheelOdomNode::jointStateCallback, this, std::placeholders::_1));
        noisyOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(
            "/nova/wheel_odom", 10);
        // Initialize parameters
        this->declare_parameter("pos_noise_stddev", 0.01);
        this->declare_parameter("rot_noise_stddev", 0.01);
        pos_stddev_ = this->get_parameter("pos_noise_stddev").as_double();
        rot_stddev_ = this->get_parameter("rot_noise_stddev").as_double();
        this->declare_parameter("wheel_base", 0.5);
        this->declare_parameter("wheel_radius", 0.1);
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        this->declare_parameter("left_wheel_joint_name", "joint_wheel_left");
        this->declare_parameter("right_wheel_joint_name", "joint_wheel_right");
        left_wheel_joint_name_ = this->get_parameter("left_wheel_joint_name").as_string();
        right_wheel_joint_name_ = this->get_parameter("right_wheel_joint_name").as_string();
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("base_link_frame_id", "base_link");
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_link_frame_id_ = this->get_parameter("base_link_frame_id").as_string();
        this->declare_parameter("tf_topic_name", "/nova/tf");
        tf_topic_name_ = this->get_parameter("tf_topic_name").as_string();
        // Initialize tf publisher
        tfPublisher = this->create_publisher<tf2_msgs::msg::TFMessage>(
            tf_topic_name_, 10);
        // Initialize random number generator
        std::random_device rd;
        gen_ = std::mt19937(rd());
        pos_dist_ = std::normal_distribution<>(0.0, pos_stddev_);
        rot_dist_ = std::normal_distribution<>(0.0, rot_stddev_);

        // Create an empty odometry message
        currentVehOdom = std::make_shared<nav_msgs::msg::Odometry>();
        // Initialize covariance matrix
        poseCovariance_ = calculateCovariance(pos_stddev_, wheel_base_, wheel_radius_);
        twistCovariance_ = calculateCovariance(rot_stddev_, wheel_base_, wheel_radius_);
    }

    // Calculate the covariance matrix for the wheel odometry from the noisy joint states
    Eigen::Matrix2d NoisyWheelOdomNode::calculateCovariance(double position_noise_stddev,
                                                 double wheel_base,
                                                 double wheel_radius)
    {
        // Propogate to odometry covariance from the joint state covariances
        // The covariance matrix is a 2x2 matrix
        // The diagonal elements are the variances of the noisy joint states
        Eigen::Matrix2d covariance;
        covariance(0, 0) = position_noise_stddev * position_noise_stddev; // Right wheel
        covariance(1, 1) = position_noise_stddev * position_noise_stddev; // Left wheel
        covariance(0, 1) = 0.0; // Right wheel and left wheel are independent
        covariance(1, 0) = 0.0; // Right wheel and left wheel are independent
        // Propogate to odometry covariance
        Eigen::Matrix2d jacobian;
        jacobian(0, 0) = wheel_radius / 2.0;
        jacobian(0, 1) = wheel_radius / 2.0;
        jacobian(1, 0) = wheel_radius / wheel_base;
        jacobian(1, 1) = -wheel_radius / wheel_base;
        // Propogate the covariance
        covariance = jacobian * covariance * jacobian.transpose();
        return covariance;
    }

    // Callback function for the joint state topic
    void NoisyWheelOdomNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Calculate local odometry from joint states
        // We need a previous joint state to calculate the change in position and orientation
        if (!currentJointState)
        {
            currentJointState = msg;
            return;
        }
        // Find the indices of the left and right wheel joints
        int left_wheel_index = -1;
        int right_wheel_index = -1;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == left_wheel_joint_name_)
            {
                left_wheel_index = i;
            }
            if (msg->name[i] == right_wheel_joint_name_)
            {
                right_wheel_index = i;
            }
        }
        if (left_wheel_index == -1 || right_wheel_index == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Left or right wheel joint not found in joint state message");
            return;
        }
        // Add noise to the joint states and velocities
        double noisy_theta_left = msg->position[left_wheel_index] + pos_dist_(gen_);
        double noisy_theta_right = msg->position[right_wheel_index] + pos_dist_(gen_);
        double noisy_velocity_left = msg->velocity[left_wheel_index] + rot_dist_(gen_);
        double noisy_velocity_right = msg->velocity[right_wheel_index] + rot_dist_(gen_);
        // Calculate the change in position and orientation
        double delta_theta_left = noisy_theta_left - currentJointState->position[left_wheel_index];
        double delta_theta_right = noisy_theta_right - currentJointState->position[right_wheel_index];
        // Calculate linear and angular velocity
        double linear_velocity = (wheel_radius_ / 2.0) * (noisy_velocity_left + noisy_velocity_right);
        double angular_velocity = (wheel_radius_ / wheel_base_) * (noisy_velocity_left - noisy_velocity_right);
        // Calculate the change in position and orientation
        double delta_distance = (wheel_radius_ / 2.0) * (delta_theta_left + delta_theta_right);
        double delta_theta = (wheel_radius_ / wheel_base_) * (delta_theta_left - delta_theta_right);
        // Publish the noisy odometry
        publishNoisyOdom(delta_distance, delta_theta, linear_velocity, angular_velocity);
        // Update the previous joint state
        currentJointState = msg;
    }

    void NoisyWheelOdomNode::publishNoisyOdom(const double& delta_distance, const double& delta_theta,
                                              const double& linear_velocity, const double& angular_velocity)
    {
        currentVehOdom->header.stamp = this->now();
        currentVehOdom->header.frame_id = odom_frame_id_;
        currentVehOdom->child_frame_id = base_link_frame_id_;
        // Get current euler angles from quaternion
        double roll, pitch, yaw;
        tf2::Quaternion q(currentVehOdom->pose.pose.orientation.x,
                          currentVehOdom->pose.pose.orientation.y,
                          currentVehOdom->pose.pose.orientation.z,
                          currentVehOdom->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // Update the pose
        currentVehOdom->pose.pose.position.x += delta_distance * std::cos(yaw + delta_theta/2);
        currentVehOdom->pose.pose.position.y += delta_distance * std::sin(yaw + delta_theta/2);
        yaw += delta_theta;
        // Convert back to a quaternion
        q.setRPY(roll, pitch, yaw);
        // Update the orientation
        currentVehOdom->pose.pose.orientation.x = q.x();
        currentVehOdom->pose.pose.orientation.y = q.y();
        currentVehOdom->pose.pose.orientation.z = q.z();
        currentVehOdom->pose.pose.orientation.w = q.w();
        // Calculate the incremental pose covariance
        Eigen::Matrix<double, 3, 2> H;
        H << std::cos(yaw + delta_theta/2), -delta_distance * std::sin(yaw + delta_theta/2),
             std::sin(yaw + delta_theta/2), delta_distance * std::cos(yaw + delta_theta/2),
             0, 1;
        Eigen::Matrix<double, 3, 3> poseCovariance = H * poseCovariance_ * H.transpose();
        double x_cov = poseCovariance(0, 0);
        double y_cov = poseCovariance(1, 1);
        double yaw_cov = poseCovariance(2, 2);
        currentVehOdom->pose.covariance[0] = x_cov;
        currentVehOdom->pose.covariance[7] = y_cov;
        currentVehOdom->pose.covariance[14] = 1000.0;
        currentVehOdom->pose.covariance[21] = 1000.0;
        currentVehOdom->pose.covariance[28] = 1000.0;
        currentVehOdom->pose.covariance[35] = yaw_cov;
        // Update the twist
        currentVehOdom->twist.twist.linear.x = linear_velocity;
        currentVehOdom->twist.twist.angular.z = angular_velocity;
        // Update the twist covariance
        currentVehOdom->twist.covariance[0] = twistCovariance_(0, 0);
        currentVehOdom->twist.covariance[7] = 1000.0;
        currentVehOdom->twist.covariance[14] = 1000.0;
        currentVehOdom->twist.covariance[21] = 1000.0;
        currentVehOdom->twist.covariance[28] = 1000.0;
        currentVehOdom->twist.covariance[35] = twistCovariance_(1, 1);
        // Publish the noisy odometry
        noisyOdomPublisher->publish(*currentVehOdom);
        // Create a tf message
        tf2_msgs::msg::TFMessage tf_msg;
        // Publish TF from odom to base_link
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = currentVehOdom->header.stamp;
        transform_msg.header.frame_id = odom_frame_id_;
        transform_msg.child_frame_id = base_link_frame_id_;
        transform_msg.transform.translation.x = currentVehOdom->pose.pose.position.x;
        transform_msg.transform.translation.y = currentVehOdom->pose.pose.position.y;
        transform_msg.transform.translation.z = currentVehOdom->pose.pose.position.z;
        transform_msg.transform.rotation = currentVehOdom->pose.pose.orientation;
        tf_msg.transforms.push_back(transform_msg);
        tfPublisher->publish(tf_msg);
    }
}


