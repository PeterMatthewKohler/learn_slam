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

    namespace error_state {
        constexpr int block_size = 3;

        constexpr int position_index = 0;
        constexpr int orientation_index = 3;    // Right-multiplicative error:
                                                // q_true = q_nominal * Exp(delta_theta)
        constexpr int velocity_index = 6;
        constexpr int gyroscope_bias_index = 9;
        constexpr int accelerometer_bias_index = 12;

        constexpr int state_size = 15;
    }

    using ErrorStateVector =
        Eigen::Matrix<double, error_state::state_size, 1>;

    using ErrorStateCovariance =
        Eigen::Matrix<
            double,
            error_state::state_size,
            error_state::state_size>;

    struct FilterState {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EstimatorState nominal_state;
        ErrorStateCovariance covariance;
    };

    struct InitialCovarianceParameters {
        double position_stddev_m;
        double orientation_stddev_rad;
        double velocity_stddev_m_s;
        double gyroscope_bias_stddev_rad_s;
        double accelerometer_bias_stddev_m_s2;
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
        FilterState filter_state;
        VisualPoseMeasurementQueue pending_visual_measurements;
    };

    struct ImuNoiseParameters {
        double gyroscope_noise_density_rad_s_sqrt_hz;       // orientation uncertainty
        double accelerometer_noise_density_m_s2_sqrt_hz;    // velocity/position uncertainty
        double gyroscope_bias_random_walk_rad_s2_sqrt_hz;   // gyroscope-bias uncertainty
        double accelerometer_bias_random_walk_m_s3_sqrt_hz; // accelerometer-bias uncertainty
    };

    namespace imu_noise {
        constexpr int block_size = 3;

        constexpr int gyroscope_index = 0;
        constexpr int accelerometer_index = 3;
        constexpr int gyroscope_bias_index = 6;
        constexpr int accelerometer_bias_index = 9;

        constexpr int noise_size = 12;
    }

    using ErrorStateDynamicsMatrix =
        Eigen::Matrix<double, error_state::state_size, error_state::state_size>;

    using ImuNoiseJacobian =
        Eigen::Matrix<double, error_state::state_size, imu_noise::noise_size>;

    using ContinuousImuNoiseCovariance =
        Eigen::Matrix<double, imu_noise::noise_size, imu_noise::noise_size>;

    struct ErrorStateLinearization {
        ErrorStateDynamicsMatrix dynamics;
        ImuNoiseJacobian noise_jacobian;
        ContinuousImuNoiseCovariance noise_covariance;
    };

    using ErrorStateTransitionMatrix =
        Eigen::Matrix<double, error_state::state_size, error_state::state_size>;

    struct DiscreteErrorStateModel {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ErrorStateTransitionMatrix transition;
        ErrorStateCovariance process_covariance;
    };

    namespace visual_pose {
        constexpr int position_index = 0;
        constexpr int orientation_index = 3;
        constexpr int residual_size = 6;
    }

    using VisualPoseResidual =
        Eigen::Matrix<double, visual_pose::residual_size, 1>;

    using VisualPoseJacobian =
        Eigen::Matrix<double, visual_pose::residual_size, error_state::state_size>;

    struct VisualPoseLinearization {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VisualPoseResidual residual;
        VisualPoseJacobian jacobian;
        Eigen::Matrix<double, 6, 6> measurement_covariance;
    };

    using VisualPoseInnovationCovariance =
        Eigen::Matrix<double, visual_pose::residual_size,
                                visual_pose::residual_size>;

    using VisualPoseKalmanGain =
        Eigen::Matrix<double, error_state::state_size,
                                visual_pose::residual_size>;

    struct VisualPoseUpdateTerms {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VisualPoseInnovationCovariance innovation_covariance;
        VisualPoseKalmanGain kalman_gain;
        ErrorStateVector error_state_correction;
    };

    struct VisualPoseCorrectionResult {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FilterState corrected_filter_state;
        VisualPoseLinearization linearization;
        VisualPoseUpdateTerms update_terms;
    };

}  // namespace vio_node

#endif  // ESTIMATOR_TYPES_HPP
