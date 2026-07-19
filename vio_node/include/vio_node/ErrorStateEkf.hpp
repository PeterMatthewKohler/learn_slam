#ifndef ERROR_STATE_EKF_HPP
#define ERROR_STATE_EKF_HPP

#include "vio_node/EstimatorTypes.hpp"

#include <optional>

namespace vio_node::error_state_ekf {
    bool validateFilterState(const FilterState& filter_state);

    std::optional<FilterState> makeInitialFilterState(
        const EstimatorState& nominal_state,
        const InitialCovarianceParameters& parameters);

    bool validateImuNoiseParameters(
        const ImuNoiseParameters& parameters);

    std::optional<ErrorStateLinearization> buildContinuousTimeLinearization(
        const Eigen::Quaterniond& world_from_imu,
        const Eigen::Vector3d& angular_velocity_unbiased_imu,
        const Eigen::Vector3d& specific_force_unbiased_imu,
        const ImuNoiseParameters& noise_parameters);

    std::optional<DiscreteErrorStateModel> discretizeErrorStateLinearization(
        const ErrorStateLinearization& linearization,
        double dt_s);

    std::optional<ErrorStateCovariance> propagateErrorStateCovariance(
        const ErrorStateCovariance& covariance,
        const DiscreteErrorStateModel& model);

    std::optional<FilterState> propagateFilterStateCovariance(
        const FilterState& current_filter_state,
        const EstimatorState& propagated_nominal_state,
        const ImuMeasurement& start_measurement,
        const ImuMeasurement& end_measurement,
        const ImuNoiseParameters& noise_parameters);

    std::optional<VisualPoseLinearization> buildVisualPoseLinearization(
        const FilterState& filter_state,
        const VisualPoseMeasurement& measurement);

    std::optional<VisualPoseUpdateTerms> computeVisualPoseUpdateTerms(
        const FilterState& filter_state,
        const VisualPoseLinearization& linearization);

    std::optional<ErrorStateCovariance> computePosteriorErrorStateCovariance(
        const ErrorStateCovariance& prior_covariance,
        const VisualPoseLinearization& linearization,
        const VisualPoseUpdateTerms& update_terms);

    std::optional<FilterState> injectErrorStateCorrection(
        const FilterState& predicted_filter_state,
        const ErrorStateVector& error_state_correction,
        const ErrorStateCovariance& posterior_covariance);

    std::optional<VisualPoseCorrectionResult> correctFilterStateWithVisualPose(
        const FilterState& predicted_filter_state,
        const VisualPoseMeasurement& measurement);

}  // namespace vio_node::error_state_ekf

#endif  // ERROR_STATE_EKF_HPP
