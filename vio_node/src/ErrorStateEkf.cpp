#include "vio_node/ErrorStateEkf.hpp"
#include "vio_node/Validation.hpp"

#include <cmath>

namespace {
    bool validStandardDeviation(double standard_deviation)
    {
        const double variance = standard_deviation * standard_deviation;
        return std::isfinite(standard_deviation) &&
            standard_deviation > 0.0 &&
            std::isfinite(variance) &&
            variance > 0.0;
    }

    bool validNoiseDensity(double noise_density)
    {
        const double squared_noise_density = noise_density * noise_density;
        return std::isfinite(noise_density) &&
            noise_density >= 0.0 &&
            std::isfinite(squared_noise_density);
    }

    // skewSymmetric(a) * b = a cross b
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vector)
    {
        Eigen::Matrix3d skew;
        skew <<
            0.0,       -vector.z(),  vector.y(),
            vector.z(), 0.0,        -vector.x(),
            -vector.y(), vector.x(),   0.0;
        return skew;
    }
}  // namespace

namespace vio_node::error_state_ekf {
    bool validateFilterState(const FilterState& filter_state)
    {
        const EstimatorState& nominal_state = filter_state.nominal_state;
        if(!validation::isFinite(nominal_state.position_world_imu) ||
           !validation::isFinite(nominal_state.velocity_world_imu) ||
           !validation::isFinite(nominal_state.gyroscope_bias) ||
           !validation::isFinite(nominal_state.accelerometer_bias) ||
           !validation::isUnitQuaternion(nominal_state.world_from_imu)) {
            return false;
        }

        const ErrorStateCovariance& covariance = filter_state.covariance;
        if(!covariance.allFinite() ||
           !covariance.isApprox(covariance.transpose(), 1e-12)) {
            return false;
        }

        const Eigen::LLT<ErrorStateCovariance> covariance_llt(covariance);
        // Validates covariance finiteness, symmetry, and positive definiteness
        return covariance_llt.info() == Eigen::Success;
    }

    std::optional<FilterState> makeInitialFilterState(
        const EstimatorState& nominal_state,
        const InitialCovarianceParameters& parameters)
    {
        if(!validStandardDeviation(parameters.position_stddev_m) ||
           !validStandardDeviation(parameters.orientation_stddev_rad) ||
           !validStandardDeviation(parameters.velocity_stddev_m_s) ||
           !validStandardDeviation(parameters.gyroscope_bias_stddev_rad_s) ||
           !validStandardDeviation(
               parameters.accelerometer_bias_stddev_m_s2)) {
            return std::nullopt;
        }

        FilterState filter_state;
        filter_state.nominal_state = nominal_state;
        filter_state.covariance.setZero();

        const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
        filter_state.covariance.block<
            error_state::block_size,
            error_state::block_size>(
                error_state::position_index,
                error_state::position_index) =
            identity * parameters.position_stddev_m *
            parameters.position_stddev_m;
        filter_state.covariance.block<
            error_state::block_size,
            error_state::block_size>(
                error_state::orientation_index,
                error_state::orientation_index) =
            identity * parameters.orientation_stddev_rad *
            parameters.orientation_stddev_rad;
        filter_state.covariance.block<
            error_state::block_size,
            error_state::block_size>(
                error_state::velocity_index,
                error_state::velocity_index) =
            identity * parameters.velocity_stddev_m_s *
            parameters.velocity_stddev_m_s;
        filter_state.covariance.block<
            error_state::block_size,
            error_state::block_size>(
                error_state::gyroscope_bias_index,
                error_state::gyroscope_bias_index) =
            identity * parameters.gyroscope_bias_stddev_rad_s *
            parameters.gyroscope_bias_stddev_rad_s;
        filter_state.covariance.block<
            error_state::block_size,
            error_state::block_size>(
                error_state::accelerometer_bias_index,
                error_state::accelerometer_bias_index) =
            identity * parameters.accelerometer_bias_stddev_m_s2 *
            parameters.accelerometer_bias_stddev_m_s2;

        if(!validateFilterState(filter_state)) {
            return std::nullopt;
        }
        return filter_state;
    }

    bool validateImuNoiseParameters(
        const ImuNoiseParameters& parameters)
    {
        return validNoiseDensity(
                   parameters.gyroscope_noise_density_rad_s_sqrt_hz) &&
            validNoiseDensity(
                   parameters.accelerometer_noise_density_m_s2_sqrt_hz) &&
            validNoiseDensity(
                   parameters.gyroscope_bias_random_walk_rad_s2_sqrt_hz) &&
            validNoiseDensity(
                   parameters.accelerometer_bias_random_walk_m_s3_sqrt_hz);
    }

    std::optional<ErrorStateLinearization> buildContinuousTimeLinearization(
        const Eigen::Quaterniond& world_from_imu,
        const Eigen::Vector3d& angular_velocity_unbiased_imu,
        const Eigen::Vector3d& specific_force_unbiased_imu,
        const ImuNoiseParameters& noise_parameters)
    {
        // validate inputs
        if(!validation::isUnitQuaternion(world_from_imu) ||
           !validation::isFinite(angular_velocity_unbiased_imu) ||
           !validation::isFinite(specific_force_unbiased_imu) ||
           !validateImuNoiseParameters(noise_parameters)){return std::nullopt;}
        // F matrix
        ErrorStateDynamicsMatrix dynamics;
        dynamics.setZero(); // Start at zero
        // Non-zero terms
        dynamics.block<error_state::block_size, error_state::block_size>
            (error_state::position_index, error_state::velocity_index) = Eigen::Matrix3d::Identity();
        dynamics.block<error_state::block_size, error_state::block_size>
            (error_state::orientation_index, error_state::orientation_index) = -skewSymmetric(angular_velocity_unbiased_imu);
        dynamics.block<error_state::block_size, error_state::block_size>
            (error_state::orientation_index, error_state::gyroscope_bias_index) = -1.0 * Eigen::Matrix3d::Identity();
        dynamics.block<error_state::block_size, error_state::block_size>
            (error_state::velocity_index, error_state::orientation_index) = -1.0 * world_from_imu.toRotationMatrix() *
                                                                                    skewSymmetric(specific_force_unbiased_imu);
        dynamics.block<error_state::block_size, error_state::block_size>
            (error_state::velocity_index, error_state::accelerometer_bias_index) = -1.0 * world_from_imu.toRotationMatrix();
        // G matrix
        ImuNoiseJacobian noise_jacobian;
        noise_jacobian.setZero(); // Start at zero
        // Non-zero terms
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::orientation_index, imu_noise::gyroscope_index) = -1.0 * Eigen::Matrix3d::Identity();
        noise_jacobian.block<error_state::block_size, imu_noise::block_sizee>
            (error_state::velocity_index, imu_noise::accelerometer_index) = -1.0 * world_from_imu.toRotationMatrix();
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::gyroscope_bias_index, imu_noise::gyroscope_bias_index) = Eigen::Matrix3d::Identity();
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::accelerometer_bias_index, imu_noise::accelerometer_bias_index) = Eigen::Matrix3d::Identity();
        // Qc - Continuous Noise Covariance
        ContinuousImuNoiseCovariance noise_covariance;
        noise_covariance.setZero(); // Start at zero
        // Non-zero terms
        noise_covariance.block<error_state::block_size, imu_noise::block_size>
            (imu_noise::gyroscope_index, imu_noise::gyroscope_index) =
                noise_parameters.gyroscope_noise_density_rad_s_sqrt_hz * noise_parameters.gyroscope_noise_density_rad_s_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        noise_covariance.block<error_state::block_size, imu_noise::block_size>
            (imu_noise::accelerometer_index, imu_noise::accelerometer_index) =
                noise_parameters.accelerometer_noise_density_m_s2_sqrt_hz * noise_parameters.accelerometer_noise_density_m_s2_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        noise_covariance.block<error_state::block_size, imu_noise::block_size>
            (imu_noise::gyroscope_bias_index, imu_noise::gyroscope_bias_index) =
                noise_parameters.gyroscope_bias_random_walk_rad_s2_sqrt_hz * noise_parameters.gyroscope_bias_random_walk_rad_s2_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        noise_covariance.block<error_state::block_size, imu_noise::block_size>
            (imu_noise::accelerometer_bias_index, imu_noise::accelerometer_bias_index) =
                noise_parameters.accelerometer_bias_random_walk_m_s3_sqrt_hz * noise_parameters.accelerometer_bias_random_walk_m_s3_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        // validate
        if(!dynamics.allFinite() ||
            !noise_jacobian.allFinite() ||
            !noise_covariance.allFinite()) {
            return std::nullopt;
        }

        return ErrorStateLinearization{
            dynamics,
            noise_jacobian,
            noise_covariance
        };

    }

}  // namespace vio_node::error_state_ekf
