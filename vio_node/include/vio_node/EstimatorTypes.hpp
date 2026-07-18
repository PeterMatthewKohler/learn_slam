#ifndef ESTIMATOR_TYPES_HPP
#define ESTIMATOR_TYPES_HPP

#include <rclcpp/time.hpp>

#include "vio_node/ImuTypes.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <deque>

namespace vio_node {
    struct EstimatorState {
        rclcpp::Time stamp;

        Eigen::Vector3d position_world_imu;
        Eigen::Quaterniond world_from_imu;
        Eigen::Vector3d velocity_world_imu;

        Eigen::Vector3d gyroscope_bias;
        Eigen::Vector3d accelerometer_bias;
    };

    struct VisualPoseMeasurement {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        rclcpp::Time stamp;

        Eigen::Vector3d position_world_imu;
        Eigen::Quaterniond world_from_imu;

        // Residual order: [position, orientation error].
        Eigen::Matrix<double, 6, 6> covariance;
    };

    using VisualPoseMeasurementQueue = std::deque<
        VisualPoseMeasurement,
        Eigen::aligned_allocator<VisualPoseMeasurement>>;

    struct EstimatorContext {
        ImuInitialization initialization;
        EstimatorState state;
        VisualPoseMeasurementQueue pending_visual_measurements;
    };
}  // namespace vio_node

#endif  // ESTIMATOR_TYPES_HPP
