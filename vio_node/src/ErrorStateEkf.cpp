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

    bool validErrorStateCovariance(const vio_node::ErrorStateCovariance& covariance)
    {
        if(!covariance.allFinite() ||
           !covariance.isApprox(covariance.transpose(), 1e-12)) {
            return false;
        }
        const Eigen::LLT<vio_node::ErrorStateCovariance> covariance_llt(covariance);
        // Validates covariance finiteness, symmetry, and positive definiteness
        return covariance_llt.info() == Eigen::Success;
    }

    bool validVisualPoseCovariance(
        const vio_node::VisualPoseInnovationCovariance& covariance)
    {
        if(!covariance.allFinite() ||
           !covariance.isApprox(covariance.transpose(), 1e-12)) {
            return false;
        }
        const Eigen::LLT<vio_node::VisualPoseInnovationCovariance>
            covariance_llt(covariance);
        return covariance_llt.info() == Eigen::Success;
    }

    bool validEstimatorState(const vio_node::EstimatorState& state)
    {
        if(!vio_node::validation::isFinite(state.position_world_imu) ||
           !vio_node::validation::isFinite(state.velocity_world_imu) ||
           !vio_node::validation::isFinite(state.gyroscope_bias) ||
           !vio_node::validation::isFinite(state.accelerometer_bias) ||
           !vio_node::validation::isUnitQuaternion(state.world_from_imu)) {
            return false;
        }
        return true;
    }
}  // namespace

namespace vio_node::error_state_ekf {
    bool validateFilterState(const FilterState& filter_state)
    {
        const EstimatorState& nominal_state = filter_state.nominal_state;
        if(!validEstimatorState(nominal_state)){return false;}

        return validErrorStateCovariance(filter_state.covariance);
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
        // F - Dynamics
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
        // G - Noise Jacobian
        ImuNoiseJacobian noise_jacobian;
        noise_jacobian.setZero(); // Start at zero
        // Non-zero terms
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::orientation_index, imu_noise::gyroscope_index) = -1.0 * Eigen::Matrix3d::Identity();
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::velocity_index, imu_noise::accelerometer_index) = -1.0 * world_from_imu.toRotationMatrix();
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::gyroscope_bias_index, imu_noise::gyroscope_bias_index) = Eigen::Matrix3d::Identity();
        noise_jacobian.block<error_state::block_size, imu_noise::block_size>
            (error_state::accelerometer_bias_index, imu_noise::accelerometer_bias_index) = Eigen::Matrix3d::Identity();
        // Qc - Continuous Noise Covariance
        ContinuousImuNoiseCovariance noise_covariance;
        noise_covariance.setZero(); // Start at zero
        // Non-zero terms
        noise_covariance.block<imu_noise::block_size, imu_noise::block_size>
            (imu_noise::gyroscope_index, imu_noise::gyroscope_index) =
                noise_parameters.gyroscope_noise_density_rad_s_sqrt_hz * noise_parameters.gyroscope_noise_density_rad_s_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        noise_covariance.block<imu_noise::block_size, imu_noise::block_size>
            (imu_noise::accelerometer_index, imu_noise::accelerometer_index) =
                noise_parameters.accelerometer_noise_density_m_s2_sqrt_hz * noise_parameters.accelerometer_noise_density_m_s2_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        noise_covariance.block<imu_noise::block_size, imu_noise::block_size>
            (imu_noise::gyroscope_bias_index, imu_noise::gyroscope_bias_index) =
                noise_parameters.gyroscope_bias_random_walk_rad_s2_sqrt_hz * noise_parameters.gyroscope_bias_random_walk_rad_s2_sqrt_hz *
                    Eigen::Matrix3d::Identity();
        noise_covariance.block<imu_noise::block_size, imu_noise::block_size>
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

    std::optional<DiscreteErrorStateModel> discretizeErrorStateLinearization(
        const ErrorStateLinearization& linearization,
        double dt_s)
    {
        // dt_s must be finite & positive
        if(!std::isfinite(dt_s) || dt_s <= 0){return std::nullopt;}
        // F, G, Qc must be finite
        if(!linearization.dynamics.allFinite() ||
           !linearization.noise_jacobian.allFinite() ||
           !linearization.noise_covariance.allFinite()){return std::nullopt;}
        // Qc must be symmetric
        if(!linearization.noise_covariance.isApprox(linearization.noise_covariance.transpose())){return std::nullopt;}
        // Discrete error model: delta_x_(k+1) = Phi * delta_x_k + process_noise
        // Phi(first order approximation) - Phi = I + F * dt
        ErrorStateTransitionMatrix Phi = ErrorStateCovariance::Identity() + linearization.dynamics * dt_s;
        // Qd = G * Qc * G^t * dt
        ErrorStateCovariance Qd = linearization.noise_jacobian * linearization.noise_covariance *
                                    linearization.noise_jacobian.transpose() * dt_s;
        // Force numerical symmetry:
        Qd = 0.5 * (Qd + Qd.transpose());
        // Validate
        // Phi & Qd are finite
        if(!Phi.allFinite() || !Qd.allFinite()){return std::nullopt;}
        // Qd symmetric
        if(!Qd.isApprox(Qd.transpose())){return std::nullopt;}
        DiscreteErrorStateModel model;
        model.transition = Phi;
        model.process_covariance = Qd;
        return model;
    }

    std::optional<ErrorStateCovariance> propagateErrorStateCovariance(
        const ErrorStateCovariance& covariance,
        const DiscreteErrorStateModel& model)
    {
        // Validate input covariance
        if(!validErrorStateCovariance(covariance)){return std::nullopt;}
        // Validate model transition matrix is finite
        if(!model.transition.allFinite()){return std::nullopt;}
        // Validate process covariance is finite and symmetric
        if(!model.process_covariance.allFinite() ||
           !model.process_covariance.isApprox(model.process_covariance.transpose())){return std::nullopt;}
        // Propagate covariance using input model
        ErrorStateCovariance prop_covariance = model.transition * covariance *
                                                model.transition.transpose() + model.process_covariance;
        // Enforce numerical symmetry
        prop_covariance = 0.5 * (prop_covariance + prop_covariance.transpose());
        // Validate and return
        if(!validErrorStateCovariance(prop_covariance)){return std::nullopt;}
        return prop_covariance;
    }

    std::optional<FilterState> propagateFilterStateCovariance(
        const FilterState& current_filter_state,
        const EstimatorState& propagated_nominal_state,
        const ImuMeasurement& start_measurement,
        const ImuMeasurement& end_measurement,
        const ImuNoiseParameters& noise_parameters)
    {
        // Validation
        // Require current filter state is valid
        if(!validateFilterState(current_filter_state)){return std::nullopt;}
        // validate estimator state
        if(!validEstimatorState(propagated_nominal_state)){return std::nullopt;}
        // Start and end measurement timestamps uses same clock
        if(!validation::useSameClock(start_measurement.stamp, end_measurement.stamp)){return std::nullopt;}
        // Current state and start_measurement use same clock and have same timestamp
        if(!validation::useSameClock(current_filter_state.nominal_state.stamp, start_measurement.stamp) ||
           current_filter_state.nominal_state.stamp != start_measurement.stamp){return std::nullopt;}
        // Propagated nominal stamp and end_measurement use same clock and have same timestamp
        if(!validation::useSameClock(propagated_nominal_state.stamp, end_measurement.stamp) ||
           propagated_nominal_state.stamp != end_measurement.stamp){return std::nullopt;}
        // Start time is before end time
        if(start_measurement.stamp >= end_measurement.stamp){return std::nullopt;}
        // Noise params are valid
        if(!validateImuNoiseParameters(noise_parameters)){return std::nullopt;}
        // dt finite and positive
        rclcpp::Duration dt = end_measurement.stamp - start_measurement.stamp;
        if(dt.seconds() <= 0 || !std::isfinite(dt.seconds())){return std::nullopt;}
        // We use midpoint values because F and G change with orientation, angular velocity, and specific force.
        // The midpoint is a better representation of the full interval than either endpoint.
        // Validate IMU measurements
        if(!start_measurement.angular_velocity.allFinite() || !start_measurement.linear_acceleration.allFinite() ||
           !end_measurement.angular_velocity.allFinite() || !end_measurement.linear_acceleration.allFinite()){return std::nullopt;}
        // Calculate unbiased midpoint measurements
        const Eigen::Vector3d angular_velocity_mid = 0.5 * (start_measurement.angular_velocity +
                                                            end_measurement.angular_velocity) -
                                                            current_filter_state.nominal_state.gyroscope_bias;

        const Eigen::Vector3d specific_force_mid = 0.5 * (start_measurement.linear_acceleration +
                                                          end_measurement.linear_acceleration) -
                                                          current_filter_state.nominal_state.accelerometer_bias;
        // Calc midpoint orientation
        Eigen::Quaterniond world_from_imu_mid =
            current_filter_state.nominal_state.world_from_imu.slerp(    // spherical linear interpolation
                0.5,
                propagated_nominal_state.world_from_imu
            );
        if(!validation::isUnitQuaternion(world_from_imu_mid)){return std::nullopt;}
        world_from_imu_mid.normalize();
        // Linearize our non-linear error state model
        const auto linearization = buildContinuousTimeLinearization(
            world_from_imu_mid,
            angular_velocity_mid,
            specific_force_mid,
            noise_parameters);
        if(!linearization){return std::nullopt;}
        // Discretize the linearized error state model
        const auto discrete_model = discretizeErrorStateLinearization(
            *linearization,
            dt.seconds());
        if(!discrete_model){return std::nullopt;}
        // Propagate covariance in our filter state
        const auto prop_covariance = propagateErrorStateCovariance(
            current_filter_state.covariance,
            *discrete_model);
        if(!prop_covariance){return std::nullopt;}
        // Build our propagated filter state
        FilterState prop_filter_state;
        prop_filter_state.nominal_state = propagated_nominal_state;
        prop_filter_state.covariance = *prop_covariance;
        // Validate
        if(!validateFilterState(prop_filter_state)){return std::nullopt;}
        return prop_filter_state;
    }

    std::optional<VisualPoseLinearization> buildVisualPoseLinearization(
        const FilterState& filter_state,
        const VisualPoseMeasurement& measurement)
    {
        // Validate
        if(!validateFilterState(filter_state)){return std::nullopt;}
        if(!validation::isFinite(measurement.position_world_imu) ||
           !validation::isUnitQuaternion(measurement.world_from_imu) ||
           !validVisualPoseCovariance(measurement.covariance)){return std::nullopt;}
        // State and measurement use same clock and have equal timestamps
        if(!validation::useSameClock(filter_state.nominal_state.stamp,
                                     measurement.stamp) ||
           filter_state.nominal_state.stamp != measurement.stamp){return std::nullopt;}
        // Build residual
        const Eigen::Vector3d position_residual =
            measurement.position_world_imu -
            filter_state.nominal_state.position_world_imu;
        Eigen::Quaterniond q_error =
            filter_state.nominal_state.world_from_imu.conjugate() *
            measurement.world_from_imu;
        // Because error is right-multiplicative
        q_error = validation::canonicalize(q_error);
        if(!validation::isUnitQuaternion(q_error)){return std::nullopt;}
        q_error.normalize();
        // Convert to rotation vector
        const Eigen::AngleAxisd angle_axis(q_error);
        const Eigen::Vector3d orientation_residual =
            angle_axis.axis() * angle_axis.angle();
        // Build complete residual
        VisualPoseResidual residual = VisualPoseResidual::Zero();
        residual.segment<3>(visual_pose::position_index) = position_residual;
        residual.segment<3>(visual_pose::orientation_index) =
            orientation_residual;
        // Build measurement Jacobian matrix H
        VisualPoseJacobian H = VisualPoseJacobian::Zero();
        // Non-zero terms
        H.block<error_state::block_size, error_state::block_size>
            (visual_pose::position_index, error_state::position_index) =
                Eigen::Matrix3d::Identity();
        H.block<error_state::block_size, error_state::block_size>
            (visual_pose::orientation_index, error_state::orientation_index) =
                Eigen::Matrix3d::Identity();
        if(!residual.allFinite() || !H.allFinite()) {
            return std::nullopt;
        }

        VisualPoseLinearization output;
        output.residual = residual;
        output.jacobian = H;
        output.measurement_covariance = measurement.covariance;
        return output;
    }

    std::optional<VisualPoseUpdateTerms> computeVisualPoseUpdateTerms(
        const FilterState& filter_state,
        const VisualPoseLinearization& linearization)
    {
        // Validate filter state
        if(!validateFilterState(filter_state)){return std::nullopt;}
        // Validate residual, H, and R
        if(!linearization.residual.allFinite() ||
           !linearization.jacobian.allFinite() ||
           !validVisualPoseCovariance(
               linearization.measurement_covariance)){return std::nullopt;}
        // Project the 15x15 state covariance into measurement space.
        // P is 15x15 and H^T is 15x6, so Pht is 15x6. It describes
        // the covariance between each error-state component and each
        // component of the six-dimensional visual pose residual.
        const Eigen::Matrix<
            double,
            error_state::state_size,
            visual_pose::residual_size> Pht =
                filter_state.covariance * linearization.jacobian.transpose();
        if(!Pht.allFinite()){return std::nullopt;}
        // Innovation covariance: S = H * P * H^T + R.
        // S is 6x6 and represents the expected uncertainty of the visual
        // residual using both predicted state uncertainty and measurement
        // uncertainty. A larger S causes the filter to trust the residual less.
        VisualPoseInnovationCovariance S =
            linearization.jacobian * Pht +
            linearization.measurement_covariance;
        // Enforce numerical symmetry before factorization
        S = 0.5 * (S + S.transpose());
        if(!validVisualPoseCovariance(S)){return std::nullopt;}
        // The Kalman gain is K = Pht * S^-1, where K is 15x6. Forming
        // S.inverse() directly would be less numerically stable and would do
        // more work than solving a linear system with the LLT factorization.
        Eigen::LLT<VisualPoseInnovationCovariance> innovation_llt(S);
        if(innovation_llt.info() != Eigen::Success){return std::nullopt;}

        // Eigen solves equations with the unknown on the right: S * X = B.
        // Transposing the Kalman gain equation gives:
        //
        //   K^T = (Pht * S^-1)^T = S^-1 * Pht^T
        //
        // because S is symmetric. Therefore we solve:
        //
        //   S * K^T = Pht^T
        //
        // The solve returns the 6x15 matrix K^T, which is transposed below.
        const Eigen::Matrix<
            double,
            visual_pose::residual_size,
            error_state::state_size> kalman_gain_transpose =
                innovation_llt.solve(Pht.transpose());
        if(innovation_llt.info() != Eigen::Success ||
           !kalman_gain_transpose.allFinite()){return std::nullopt;}
        const VisualPoseKalmanGain K = kalman_gain_transpose.transpose();
        // Apply Kalman gain to get state correction: dx = K * r
        const ErrorStateVector dx = K * linearization.residual;
        if(!K.allFinite() || !dx.allFinite()){return std::nullopt;}

        VisualPoseUpdateTerms output;
        output.innovation_covariance = S;
        output.kalman_gain = K;
        output.error_state_correction = dx;
        return output;
    }

    std::optional<ErrorStateCovariance> computePosteriorErrorStateCovariance(
        const ErrorStateCovariance& prior_covariance,
        const VisualPoseLinearization& linearization,
        const VisualPoseUpdateTerms& update_terms)
    {
        // Validate
        // prior_covariance is finite, symmetric and positive-definite
        if(!validErrorStateCovariance(prior_covariance)){return std::nullopt;}
        // Jacobian(H) and Kalman gain(K) are finite
        if(!linearization.jacobian.allFinite() ||
           !update_terms.kalman_gain.allFinite()){return std::nullopt;}
        // Validate measurement covariance
        if(!validVisualPoseCovariance(linearization.measurement_covariance)){return std::nullopt;}
        // innovation covariance(S) is finite, symmetric, and positive-definite
        if(!validVisualPoseCovariance(update_terms.innovation_covariance)) {return std::nullopt;}
        // Error state correction (delta_x) is valid
        if(!update_terms.error_state_correction.allFinite()){return std::nullopt;}
        // Calculate update matrix: A = I - K * H
        const ErrorStateCovariance update_matrix =
            ErrorStateCovariance::Identity() -
            update_terms.kalman_gain * linearization.jacobian;
        if(!update_matrix.allFinite()){return std::nullopt;}
        // Calculate posterior error state covariance (P_post)
        // P_post = A * P_prior * A^T + K * R * K^T  <-- Joseph Form
        // Joseph form better preserves covariance symmetry and positive
        // semidefiniteness under floating point error.
        ErrorStateCovariance posterior_covariance =
            update_matrix * prior_covariance * update_matrix.transpose() +
            update_terms.kalman_gain * linearization.measurement_covariance *
                update_terms.kalman_gain.transpose();
        // Enforce numerical symmetry
        posterior_covariance = 0.5 * (posterior_covariance +
                               posterior_covariance.transpose());
        // Validate posterior covariance
        if(!validErrorStateCovariance(posterior_covariance)){return std::nullopt;}
        return posterior_covariance;
    }

    std::optional<FilterState> injectErrorStateCorrection(
        const FilterState& predicted_filter_state,
        const ErrorStateVector& error_state_correction,
        const ErrorStateCovariance& posterior_covariance)
    {
        // Validate input filter state and posterior covariance
        if(!validateFilterState(predicted_filter_state) ||
           !validErrorStateCovariance(posterior_covariance)){return std::nullopt;}
        // Validate corrections
        if(!error_state_correction.allFinite()){return std::nullopt;}
        // Get our correction blocks
        Eigen::Vector3d delta_position, delta_orientation, delta_velocity,
                        delta_gyroscope_bias, delta_accelerometer_bias;
        delta_position = error_state_correction.segment<error_state::block_size>
                            (error_state::position_index);
        delta_orientation = error_state_correction.segment<error_state::block_size>
                                (error_state::orientation_index);
        delta_velocity = error_state_correction.segment<error_state::block_size>
                            (error_state::velocity_index);
        delta_gyroscope_bias = error_state_correction.segment<error_state::block_size>
                                (error_state::gyroscope_bias_index);
        delta_accelerometer_bias = error_state_correction.segment<error_state::block_size>
                                    (error_state::accelerometer_bias_index);
        // Apply additive components
        FilterState corrected_filter_state = predicted_filter_state;
        // Position
        corrected_filter_state.nominal_state.position_world_imu =
            predicted_filter_state.nominal_state.position_world_imu + delta_position;
        // Orientation: convert into a quaternion then use right-multiplicative convention
        double delta_angle = delta_orientation.norm();
        Eigen::Quaterniond delta_quaternion;
        if(!std::isfinite(delta_angle)) {
            return std::nullopt;
        }
        if(delta_angle < 1e-8) {
            delta_quaternion = Eigen::Quaterniond(
                1.0,
                0.5 * delta_orientation.x(),
                0.5 * delta_orientation.y(),
                0.5 * delta_orientation.z()
            );
        }
        else {
            delta_quaternion = Eigen::Quaterniond(
                Eigen::AngleAxisd(delta_angle, delta_orientation / delta_angle)
            );
        }
        const double delta_squared_norm = delta_quaternion.squaredNorm();
        if(!validation::isFinite(delta_quaternion) ||
           !std::isfinite(delta_squared_norm) ||
           delta_squared_norm < 1e-12) {
            return std::nullopt;
        }
        delta_quaternion.normalize();

        Eigen::Quaterniond corrected_orientation =
            predicted_filter_state.nominal_state.world_from_imu *
            delta_quaternion;
        const double corrected_squared_norm =
            corrected_orientation.squaredNorm();
        if(!validation::isFinite(corrected_orientation) ||
           !std::isfinite(corrected_squared_norm) ||
           corrected_squared_norm < 1e-12) {
            return std::nullopt;
        }
        corrected_orientation.normalize();
        corrected_filter_state.nominal_state.world_from_imu =
            corrected_orientation;
        // Velocity
        corrected_filter_state.nominal_state.velocity_world_imu =
            predicted_filter_state.nominal_state.velocity_world_imu + delta_velocity;
        // Gyro Bias
        corrected_filter_state.nominal_state.gyroscope_bias =
            predicted_filter_state.nominal_state.gyroscope_bias + delta_gyroscope_bias;
        // Accelerometer Bias
        corrected_filter_state.nominal_state.accelerometer_bias =
            predicted_filter_state.nominal_state.accelerometer_bias + delta_accelerometer_bias;

        // Injecting the estimated error makes the corrected nominal state our new
        // reference, so the error-state mean is reset to zero. This does not erase
        // uncertainty; it re-expresses the posterior covariance around that new reference.
        // Additive state components keep the same coordinates, but the right-multiplicative
        // orientation error needs this first-order reset Jacobian after the quaternion update.
        ErrorStateTransitionMatrix reset_jacobian =
            ErrorStateTransitionMatrix::Identity();
        reset_jacobian.block<error_state::block_size, error_state::block_size>
            (error_state::orientation_index, error_state::orientation_index) =
                Eigen::Matrix3d::Identity() -
                0.5 * skewSymmetric(delta_orientation);
        // Validate reset jacobian
        if(!reset_jacobian.allFinite()){return std::nullopt;}
        ErrorStateCovariance reset_covariance = reset_jacobian *
                                                posterior_covariance *
                                                reset_jacobian.transpose();
        // Enforce covariance symmetry and validate
        reset_covariance = 0.5 * (reset_covariance + reset_covariance.transpose());
        if(!validErrorStateCovariance(reset_covariance)){return std::nullopt;}
        corrected_filter_state.covariance = reset_covariance;
        // Validate output
        if(!validateFilterState(corrected_filter_state)){return std::nullopt;}
        return corrected_filter_state;
    }

    std::optional<VisualPoseCorrectionResult> correctFilterStateWithVisualPose(
        const FilterState& predicted_filter_state,
        const VisualPoseMeasurement& measurement)
    {
        // Linearize the visual measurement model around the predicted state
        const auto linearization = buildVisualPoseLinearization(
            predicted_filter_state,
            measurement
        );
        if(!linearization){return std::nullopt;}
        // Ensure linearization terms remain finite
        if(!linearization->residual.allFinite() ||
           !linearization->jacobian.allFinite() ||
           !validVisualPoseCovariance(
               linearization->measurement_covariance)){return std::nullopt;}
        // Calculate our innovation covariance, kalman gain and state corrections
        const auto update_terms = computeVisualPoseUpdateTerms(
            predicted_filter_state,
            *linearization
        );
        if(!update_terms){return std::nullopt;}
        if(!validVisualPoseCovariance(
               update_terms->innovation_covariance) ||
           !update_terms->kalman_gain.allFinite() ||
           !update_terms->error_state_correction.allFinite()){return std::nullopt;}
        // Calculate our posterior covariance
        const auto posterior_covariance = computePosteriorErrorStateCovariance(
            predicted_filter_state.covariance,
            *linearization,
            *update_terms
        );
        if(!posterior_covariance){return std::nullopt;}
        // Compute our corrected filter state
        const auto corrected_filter_state = injectErrorStateCorrection(
            predicted_filter_state,
            update_terms->error_state_correction,
            *posterior_covariance
        );
        if(!corrected_filter_state){return std::nullopt;}
        // Validate
        if(!validateFilterState(*corrected_filter_state) ||
           !validation::useSameClock(
               corrected_filter_state->nominal_state.stamp,
               measurement.stamp) ||
           corrected_filter_state->nominal_state.stamp != measurement.stamp) {
            return std::nullopt;
        }

        VisualPoseCorrectionResult result;
        result.corrected_filter_state = *corrected_filter_state;
        result.linearization = *linearization;
        result.update_terms = *update_terms;
        return result;
    }

}  // namespace vio_node::error_state_ekf
