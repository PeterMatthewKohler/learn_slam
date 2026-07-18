#ifndef IMU_TYPES_HPP
#define IMU_TYPES_HPP

#include <rclcpp/time.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstddef>

namespace vio_node {
    struct ImuMeasurement {
        rclcpp::Time stamp;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
    };

    struct ImuWindowStatistics {
        std::size_t sample_count;
        double duration_s;

        Eigen::Vector3d angular_velocity_mean;
        Eigen::Vector3d angular_velocity_stddev;

        Eigen::Vector3d linear_acceleration_mean;
        Eigen::Vector3d linear_acceleration_stddev;
    };

    struct ImuInitialization {
        rclcpp::Time stamp;

        Eigen::Vector3d gyroscope_bias;
        Eigen::Vector3d accelerometer_bias;
        Eigen::Quaterniond world_from_imu;
        Eigen::Vector3d gravity_world;
    };
}  // namespace vio_node

#endif  // IMU_TYPES_HPP
