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

}  // namespace vio_node::error_state_ekf
