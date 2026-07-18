#ifndef ERROR_STATE_EKF_HPP
#define ERROR_STATE_EKF_HPP

#include "vio_node/EstimatorTypes.hpp"

#include <optional>

namespace vio_node::error_state_ekf {
    bool validateFilterState(const FilterState& filter_state);

    std::optional<FilterState> makeInitialFilterState(
        const EstimatorState& nominal_state,
        const InitialCovarianceParameters& parameters);

}  // namespace vio_node::error_state_ekf

#endif  // ERROR_STATE_EKF_HPP
