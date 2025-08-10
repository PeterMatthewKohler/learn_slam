#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <optional>
#include <random>

namespace noisy_wheel_odom {
    class NoisyWheelOdomNode : public rclcpp::Node
    {
        public:
        NoisyWheelOdomNode(const rclcpp::NodeOptions& options);

        private:
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisyOdomPublisher;
        void publishNoisyOdom(const double& delta_distance, const double& delta_theta,
                              const double& linear_velocity, const double& angular_velocity);
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPublisher;
        // Internal states
        sensor_msgs::msg::JointState::SharedPtr currentJointState;
        nav_msgs::msg::Odometry::SharedPtr currentVehOdom;
        sensor_msgs::msg::Imu::SharedPtr currentImu;
        // Mutex for thread safety
        std::mutex mutex_;
        // Parameters
        double pos_stddev_{};
        double rot_stddev_{};
        std::string odom_frame_id_{};
        std::string base_link_frame_id_{};
        std::string tf_topic_name_{};
        // Random number generator
        std::mt19937 gen_;
        std::normal_distribution<> pos_dist_;
        std::normal_distribution<> rot_dist_;
        // Robot parameters
        double wheel_base_{};
        double wheel_radius_{};
        std::string left_wheel_joint_name_{};
        std::string right_wheel_joint_name_{};
        // Covariance matrix
        Eigen::Matrix2d poseCovariance_;
        Eigen::Matrix2d twistCovariance_;

        // Functions
        Eigen::Matrix2d calculateCovariance(double position_noise_stddev,
                                             double wheel_base,
                                             double wheel_radius);

        
    };
}


