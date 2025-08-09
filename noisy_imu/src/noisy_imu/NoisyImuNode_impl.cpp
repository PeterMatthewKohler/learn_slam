#include "noisy_imu/NoisyImuNode_impl.hpp"
#include <cmath>
#include <functional>

namespace noisy_imu {

NoisyImuNode::NoisyImuNode(const rclcpp::NodeOptions &options)
    : Node("noisy_imu_node", options)
{
    this->declare_parameter("orientation_noise_stddev", 0.001);
    this->declare_parameter("angular_velocity_noise_stddev", 0.001);
    this->declare_parameter("linear_acceleration_noise_stddev", 0.01);

    orientationStddev_ = this->get_parameter("orientation_noise_stddev").as_double();
    angularVelocityStddev_ = this->get_parameter("angular_velocity_noise_stddev").as_double();
    linearAccelerationStddev_ = this->get_parameter("linear_acceleration_noise_stddev").as_double();

    std::random_device rd;
    randomGenerator_ = std::mt19937(rd());
    orientationNoiseDist_ = std::normal_distribution<>(0.0, orientationStddev_);
    angularVelocityNoiseDist_ = std::normal_distribution<>(0.0, angularVelocityStddev_);
    linearAccelerationNoiseDist_ = std::normal_distribution<>(0.0, linearAccelerationStddev_);

    imuSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        std::bind(&NoisyImuNode::imuCallback, this, std::placeholders::_1));

    noisyImuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/noisy", rclcpp::SensorDataQoS());
}

void NoisyImuNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto noisy = sensor_msgs::msg::Imu(*msg);

    noisy.orientation.x += orientationNoiseDist_(randomGenerator_);
    noisy.orientation.y += orientationNoiseDist_(randomGenerator_);
    noisy.orientation.z += orientationNoiseDist_(randomGenerator_);
    noisy.orientation.w += orientationNoiseDist_(randomGenerator_);
    double norm = std::sqrt(
        noisy.orientation.x * noisy.orientation.x +
        noisy.orientation.y * noisy.orientation.y +
        noisy.orientation.z * noisy.orientation.z +
        noisy.orientation.w * noisy.orientation.w);
    if (norm > 1e-9) {
        noisy.orientation.x /= norm;
        noisy.orientation.y /= norm;
        noisy.orientation.z /= norm;
        noisy.orientation.w /= norm;
    }
    if (noisy.orientation_covariance[0] >= 0.0) {
        for (int i = 0; i < 3; ++i) {
            noisy.orientation_covariance[i * 3 + i] += orientationStddev_ * orientationStddev_;
        }
    }

    noisy.angular_velocity.x += angularVelocityNoiseDist_(randomGenerator_);
    noisy.angular_velocity.y += angularVelocityNoiseDist_(randomGenerator_);
    noisy.angular_velocity.z += angularVelocityNoiseDist_(randomGenerator_);
    if (noisy.angular_velocity_covariance[0] >= 0.0) {
        for (int i = 0; i < 3; ++i) {
            noisy.angular_velocity_covariance[i * 3 + i] += angularVelocityStddev_ * angularVelocityStddev_;
        }
    }

    noisy.linear_acceleration.x += linearAccelerationNoiseDist_(randomGenerator_);
    noisy.linear_acceleration.y += linearAccelerationNoiseDist_(randomGenerator_);
    noisy.linear_acceleration.z += linearAccelerationNoiseDist_(randomGenerator_);
    if (noisy.linear_acceleration_covariance[0] >= 0.0) {
        for (int i = 0; i < 3; ++i) {
            noisy.linear_acceleration_covariance[i * 3 + i] += linearAccelerationStddev_ * linearAccelerationStddev_;
        }
    }

    noisy.header.stamp = this->get_clock()->now();
    noisy.header.frame_id = msg->header.frame_id;

    noisyImuPublisher_->publish(noisy);
}

} // namespace noisy_imu


