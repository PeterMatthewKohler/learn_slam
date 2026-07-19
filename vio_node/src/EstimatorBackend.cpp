#include <vio_node/VIONode.hpp>
#include <vio_node/ErrorStateEkf.hpp>
#include <vio_node/Validation.hpp>

#include <cmath>

namespace vio_node {
    std::optional<EstimatorState> VIONode::makeInitialEstimatorState(
        const ImuInitialization& initialization) const
    {
        if(!validation::isFinite(initialization.gyroscope_bias) ||
           !validation::isFinite(initialization.accelerometer_bias) ||
           !validation::isFinite(initialization.gravity_world) ||
           !validation::isUnitQuaternion(initialization.world_from_imu)) {
            return std::nullopt;
        }

        EstimatorState state;
        state.stamp = initialization.stamp;
        state.position_world_imu = Eigen::Vector3d::Zero();
        state.world_from_imu = initialization.world_from_imu;
        state.velocity_world_imu = Eigen::Vector3d::Zero();
        state.gyroscope_bias = initialization.gyroscope_bias;
        state.accelerometer_bias = initialization.accelerometer_bias;
        return state;
    }

    std::optional<EstimatorState> VIONode::propagateEstimatorState(
        const EstimatorState& state,
        const ImuMeasurement& start_measurement,
        const ImuMeasurement& end_measurement,
        const Eigen::Vector3d& gravity_world) const
    {
        if(!validation::useSameClock(state.stamp, start_measurement.stamp) ||
           !validation::useSameClock(state.stamp, end_measurement.stamp) ||
           state.stamp != start_measurement.stamp ||
           start_measurement.stamp >= end_measurement.stamp) {
            return std::nullopt;
        }

        const double dt_s =
            (end_measurement.stamp - start_measurement.stamp).seconds();
        if(!std::isfinite(dt_s) ||
           dt_s <= 0.0 ||
           dt_s > imuMsgGapThresholdS_) {
            return std::nullopt;
        }

        if(!validation::isFinite(state.position_world_imu) ||
           !validation::isFinite(state.velocity_world_imu) ||
           !validation::isFinite(state.gyroscope_bias) ||
           !validation::isFinite(state.accelerometer_bias) ||
           !validation::isUnitQuaternion(state.world_from_imu) ||
           !validation::isFinite(start_measurement.angular_velocity) ||
           !validation::isFinite(start_measurement.linear_acceleration) ||
           !validation::isFinite(end_measurement.angular_velocity) ||
           !validation::isFinite(end_measurement.linear_acceleration) ||
           !validation::isFinite(gravity_world)) {
            return std::nullopt;
        }

        EstimatorState next;
        next.stamp = end_measurement.stamp;
        next.gyroscope_bias = state.gyroscope_bias;
        next.accelerometer_bias = state.accelerometer_bias;

        const Eigen::Vector3d angular_velocity_mid =
            0.5 * (start_measurement.angular_velocity +
                   end_measurement.angular_velocity) -
            state.gyroscope_bias;
        const Eigen::Vector3d delta_angle = angular_velocity_mid * dt_s;
        if(!validation::isFinite(angular_velocity_mid) ||
           !validation::isFinite(delta_angle)) {
            return std::nullopt;
        }

        Eigen::Quaterniond delta_quaternion;
        const double angle = delta_angle.norm();
        if(!std::isfinite(angle)) {
            return std::nullopt;
        }
        if(angle < 1e-8) {
            delta_quaternion = Eigen::Quaterniond(
                1.0,
                0.5 * delta_angle.x(),
                0.5 * delta_angle.y(),
                0.5 * delta_angle.z()
            );
        }
        else {
            delta_quaternion = Eigen::Quaterniond(
                Eigen::AngleAxisd(angle, delta_angle / angle)
            );
        }

        const double delta_squared_norm = delta_quaternion.squaredNorm();
        if(!validation::isFinite(delta_quaternion) ||
           !std::isfinite(delta_squared_norm) ||
           delta_squared_norm < 1e-12) {
            return std::nullopt;
        }
        delta_quaternion.normalize();

        Eigen::Quaterniond world_from_imu_next =
            state.world_from_imu * delta_quaternion;
        const double next_squared_norm = world_from_imu_next.squaredNorm();
        if(!validation::isFinite(world_from_imu_next) ||
           !std::isfinite(next_squared_norm) ||
           next_squared_norm < 1e-12) {
            return std::nullopt;
        }
        world_from_imu_next.normalize();
        next.world_from_imu = world_from_imu_next;

        const Eigen::Vector3d acceleration_start_world =
            state.world_from_imu *
                (start_measurement.linear_acceleration -
                 state.accelerometer_bias) +
            gravity_world;
        const Eigen::Vector3d acceleration_end_world =
            world_from_imu_next *
                (end_measurement.linear_acceleration -
                 state.accelerometer_bias) +
            gravity_world;
        const Eigen::Vector3d acceleration_mid_world =
            0.5 * (acceleration_start_world + acceleration_end_world);
        if(!validation::isFinite(acceleration_start_world) ||
           !validation::isFinite(acceleration_end_world) ||
           !validation::isFinite(acceleration_mid_world)) {
            return std::nullopt;
        }

        next.position_world_imu =
            state.position_world_imu +
            state.velocity_world_imu * dt_s +
            0.5 * acceleration_mid_world * dt_s * dt_s;
        next.velocity_world_imu =
            state.velocity_world_imu + acceleration_mid_world * dt_s;

        if(!validation::isFinite(next.position_world_imu) ||
           !validation::isFinite(next.velocity_world_imu) ||
           !validation::isFinite(next.gyroscope_bias) ||
           !validation::isFinite(next.accelerometer_bias) ||
           !validation::isUnitQuaternion(next.world_from_imu)) {
            return std::nullopt;
        }
        return next;
    }

  std::optional<FilterState> VIONode::propagateEstimatorStateThroughMeasurements(
      const FilterState& initial_filter_state,
      const std::vector<ImuMeasurement>& measurements,
      const Eigen::Vector3d& gravity_world) const
    {
        // Validation
        if(measurements.size() < std::size_t(2) ||
           !validation::useSameClock(measurements.front().stamp, initial_filter_state.nominal_state.stamp) ||
           measurements.front().stamp != initial_filter_state.nominal_state.stamp) {return std::nullopt;}
        if(!error_state_ekf::validateFilterState(initial_filter_state)){return std::nullopt;}
        // Initialize
        FilterState propagated_filter_state = initial_filter_state;
        // Propagate through every IMU interval
        for(std::size_t i = 1; i < measurements.size(); i++) {
            // Propagate estimator state
            const auto estimator_state = propagateEstimatorState(
                propagated_filter_state.nominal_state,
                measurements[i-1],
                measurements[i],
                gravity_world);
            if(!estimator_state){return std::nullopt;}
            // Propagate state covariance matrix
            const auto updated_filter = error_state_ekf::propagateFilterStateCovariance(
                propagated_filter_state,
                *estimator_state,
                measurements[i-1],
                measurements[i],
                imuNoiseParameters_);
            // If successful, assign back to filter state
            if(!updated_filter){return std::nullopt;}
            propagated_filter_state = *updated_filter;
        }
        // Validate before returning
        if(!error_state_ekf::validateFilterState(propagated_filter_state)){return std::nullopt;}
        // Final propagated state matches final imu measurement timestamp
        if(propagated_filter_state.nominal_state.stamp != measurements.back().stamp){return std::nullopt;}
        return propagated_filter_state;
    }

    std::optional<VisualCameraPose> VIONode::visualCameraPoseFromEstimatorState(
        const EstimatorState& state,
        const geometry_msgs::msg::TransformStamped& imu_from_camera) const
    {
        if(!validation::isFinite(state.position_world_imu) ||
           !validation::isUnitQuaternion(state.world_from_imu)) {
            return std::nullopt;
        }

        const Eigen::Matrix3d rotation_world_from_imu_eigen =
            state.world_from_imu.toRotationMatrix();
        const cv::Matx33d rotation_world_from_imu(
            rotation_world_from_imu_eigen(0, 0),
            rotation_world_from_imu_eigen(0, 1),
            rotation_world_from_imu_eigen(0, 2),
            rotation_world_from_imu_eigen(1, 0),
            rotation_world_from_imu_eigen(1, 1),
            rotation_world_from_imu_eigen(1, 2),
            rotation_world_from_imu_eigen(2, 0),
            rotation_world_from_imu_eigen(2, 1),
            rotation_world_from_imu_eigen(2, 2)
        );
        const cv::Vec3d translation_world_from_imu(
            state.position_world_imu.x(),
            state.position_world_imu.y(),
            state.position_world_imu.z()
        );

        cv::Matx33d rotation_imu_from_camera;
        cv::Vec3d translation_imu_from_camera;
        if(!transformToOpenCV(
               imu_from_camera,
               rotation_imu_from_camera,
               translation_imu_from_camera)) {
            return std::nullopt;
        }

        const cv::Matx33d rotation_world_from_camera =
            rotation_world_from_imu * rotation_imu_from_camera;
        const cv::Vec3d translation_world_from_camera =
            translation_world_from_imu +
            rotation_world_from_imu * translation_imu_from_camera;
        if(!validation::isFinite(translation_world_from_camera) ||
           !validation::isRotationMatrix(rotation_world_from_camera)) {
            return std::nullopt;
        }

        return VisualCameraPose{
            rotation_world_from_camera,
            translation_world_from_camera
        };
    }

    std::optional<VisualPoseMeasurement>
    VIONode::visualPoseMeasurementFromCameraPose(
        const VisualCameraPose& world_from_camera,
        const rclcpp::Time& stamp,
        const geometry_msgs::msg::TransformStamped& imu_from_camera) const
    {
        if(imu_from_camera.header.frame_id != imuFrameID_ ||
           imu_from_camera.child_frame_id != leftCameraFrameID_ ||
           !validation::isFinite(
               world_from_camera.translation_world_from_camera) ||
           !validation::isRotationMatrix(
               world_from_camera.rotation_world_from_camera)) {
            return std::nullopt;
        }

        cv::Matx33d rotation_imu_from_camera;
        cv::Vec3d translation_imu_from_camera;
        if(!transformToOpenCV(
               imu_from_camera,
               rotation_imu_from_camera,
               translation_imu_from_camera)) {
            return std::nullopt;
        }

        const cv::Matx33d rotation_camera_from_imu =
            rotation_imu_from_camera.t();
        const cv::Vec3d translation_camera_from_imu =
            -(rotation_camera_from_imu * translation_imu_from_camera);
        const cv::Matx33d rotation_world_from_imu =
            world_from_camera.rotation_world_from_camera *
            rotation_camera_from_imu;
        const cv::Vec3d translation_world_from_imu =
            world_from_camera.translation_world_from_camera +
            world_from_camera.rotation_world_from_camera *
                translation_camera_from_imu;

        const auto orientation =
            quaternionFromRotationMatrix(rotation_world_from_imu);
        if(!validation::isFinite(translation_world_from_imu) ||
           !orientation) {
            return std::nullopt;
        }

        const double position_variance =
            visualPositionStddevM_ * visualPositionStddevM_;
        const double orientation_variance =
            visualOrientationStddevRad_ * visualOrientationStddevRad_;
        if(!std::isfinite(position_variance) ||
           !std::isfinite(orientation_variance) ||
           position_variance <= 0.0 ||
           orientation_variance <= 0.0) {
            return std::nullopt;
        }

        VisualPoseMeasurement measurement;
        measurement.stamp = stamp;
        measurement.position_world_imu = Eigen::Vector3d(
            translation_world_from_imu[0],
            translation_world_from_imu[1],
            translation_world_from_imu[2]
        );
        measurement.world_from_imu = Eigen::Quaterniond(
            orientation->w,
            orientation->x,
            orientation->y,
            orientation->z
        );
        if(measurement.world_from_imu.w() < 0.0) {
            measurement.world_from_imu.coeffs() *= -1.0;
        }

        measurement.covariance.setZero();
        measurement.covariance.block<3, 3>(0, 0) =
            Eigen::Matrix3d::Identity() * position_variance;
        measurement.covariance.block<3, 3>(3, 3) =
            Eigen::Matrix3d::Identity() * orientation_variance;

        if(!validateVisualPoseMeasurement(measurement)) {
            return std::nullopt;
        }
        return measurement;
    }

    bool VIONode::validateVisualPoseMeasurement(
        const VisualPoseMeasurement& measurement) const
    {
        if(!validation::isFinite(measurement.position_world_imu) ||
           !validation::isUnitQuaternion(measurement.world_from_imu) ||
           !measurement.covariance.allFinite() ||
           !measurement.covariance.isApprox(
               measurement.covariance.transpose(),
               1e-12)) {
            return false;
        }

        const Eigen::LLT<Eigen::Matrix<double, 6, 6>> covariance_llt(
            measurement.covariance
        );
        return covariance_llt.info() == Eigen::Success;
    }

    bool VIONode::enqueueVisualPoseMeasurement(
        const VisualPoseMeasurement& measurement)
    {
        if(!validateVisualPoseMeasurement(measurement)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Rejected invalid visual pose measurement"
            );
            return false;
        }

        std::lock_guard<std::mutex> lock(dataMutex_);
        if(!estimatorContext_) {
            return false;
        }

        auto& state = estimatorContext_->filter_state.nominal_state;
        auto& pending = estimatorContext_->pending_visual_measurements;
        if(!validation::useSameClock(measurement.stamp, state.stamp)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Rejected visual measurement using a different ROS clock than the estimator state"
            );
            return false;
        }
        if(measurement.stamp <= state.stamp) {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Skipped visual measurement at %.9f s because estimator state is already at %.9f s",
                measurement.stamp.seconds(),
                state.stamp.seconds()
            );
            return false;
        }

        if(!pending.empty() &&
           (!validation::useSameClock(
                pending.back().stamp,
                measurement.stamp) ||
            measurement.stamp <= pending.back().stamp)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Rejected non-increasing or clock-mismatched visual measurement timestamp"
            );
            return false;
        }

        if(pending.size() >= visualMeasurementQueueMaxSize_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Visual measurement queue reached %zu entries; dropping oldest measurement",
                visualMeasurementQueueMaxSize_
            );
            pending.pop_front();
        }

        pending.push_back(measurement);
        return true;
    }


    void VIONode::processPendingVisualMeasurements()
    {
        while(true) {
            std::deque<sensor_msgs::msg::Imu> buffer;
            std::optional<ImuInitialization> initialization;
            std::optional<FilterState> current_filter_state;
            std::optional<VisualPoseMeasurement> measurement;
            std::size_t pending_measurement_count = 0;
            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                if(!estimatorContext_ ||
                   estimatorContext_->pending_visual_measurements.empty()) {
                    return;
                }

                buffer = imuBuffer_;
                initialization = estimatorContext_->initialization;
                current_filter_state = estimatorContext_->filter_state;
                measurement = estimatorContext_->pending_visual_measurements.front();
                pending_measurement_count = estimatorContext_->pending_visual_measurements.size();
            }

            if(!error_state_ekf::validateFilterState(*current_filter_state)) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Cannot process visual measurement with an invalid filter state"
                );
                return;
            }
            const EstimatorState& current_state = current_filter_state->nominal_state;

            if(!validateVisualPoseMeasurement(*measurement)) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Dropping invalid measurement from visual measurement queue"
                );
                std::lock_guard<std::mutex> lock(dataMutex_);
                if(estimatorContext_ &&
                   !estimatorContext_->pending_visual_measurements.empty() &&
                   estimatorContext_->pending_visual_measurements.front().stamp ==
                       measurement->stamp) {
                    estimatorContext_->pending_visual_measurements.pop_front();
                }
                continue;
            }
            if(!validation::useSameClock(measurement->stamp, current_state.stamp)) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Dropping visual measurement using a different ROS clock than the estimator state"
                );
                std::lock_guard<std::mutex> lock(dataMutex_);
                if(estimatorContext_ &&
                   !estimatorContext_->pending_visual_measurements.empty() &&
                   validation::useSameClock(
                       estimatorContext_->pending_visual_measurements.front().stamp,
                       measurement->stamp) &&
                   estimatorContext_->pending_visual_measurements.front()
                           .stamp.nanoseconds() ==
                       measurement->stamp.nanoseconds()) {
                    estimatorContext_->pending_visual_measurements.pop_front();
                }
                continue;
            }
            if(measurement->stamp <= current_state.stamp) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Dropping stale visual measurement at %.9f s; estimator state is at %.9f s",
                    measurement->stamp.seconds(),
                    current_state.stamp.seconds()
                );
                std::lock_guard<std::mutex> lock(dataMutex_);
                if(estimatorContext_ &&
                   !estimatorContext_->pending_visual_measurements.empty() &&
                   estimatorContext_->pending_visual_measurements.front().stamp ==
                       measurement->stamp) {
                    estimatorContext_->pending_visual_measurements.pop_front();
                }
                continue;
            }
            if(buffer.size() < std::size_t(2)) {
                return;
            }

            const rclcpp::Time oldest_imu_stamp(buffer.back().header.stamp);
            const rclcpp::Time newest_imu_stamp(buffer.front().header.stamp);
            if(!validation::useSameClock(
                   oldest_imu_stamp,
                   current_state.stamp) ||
               !validation::useSameClock(
                   newest_imu_stamp,
                   current_state.stamp)) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Cannot process visual measurement because IMU and estimator timestamps use different ROS clocks"
                );
                return;
            }
            if(newest_imu_stamp < measurement->stamp) {
                RCLCPP_DEBUG_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Waiting for IMU coverage of visual measurement: visual=%.9f s, newest_imu=%.9f s, pending=%zu",
                    measurement->stamp.seconds(),
                    newest_imu_stamp.seconds(),
                    pending_measurement_count
                );
                return;
            }
            if(oldest_imu_stamp > current_state.stamp) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Cannot propagate estimator from %.9f s because oldest buffered IMU sample is %.9f s",
                    current_state.stamp.seconds(),
                    oldest_imu_stamp.seconds()
                );
                return;
            }

            const auto imu_measurements = extractImuMeasurements(
                buffer,
                current_state.stamp,
                measurement->stamp
            );
            if(!imu_measurements) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "IMU extraction failed for covered visual interval [%.9f, %.9f] s",
                    current_state.stamp.seconds(),
                    measurement->stamp.seconds()
                );
                return;
            }

            const auto propagated_filter_state =
                propagateEstimatorStateThroughMeasurements(*current_filter_state,
                                                           *imu_measurements,
                                                           initialization->gravity_world);
            if(!propagated_filter_state ||
                propagated_filter_state->nominal_state.stamp != measurement->stamp) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Estimator propagation to pending visual measurement at %.9f s failed",
                    measurement->stamp.seconds());
                return;
            }

            // The EKF visual correction will be applied here. Both the nominal
            // state and visual measurement now describe the same timestamp.
            bool handoff_stored = false;
            bool stale_snapshot_detected = false;
            std::size_t remaining_measurements = 0;
            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                if(!estimatorContext_ ||
                   estimatorContext_->filter_state.nominal_state.stamp !=
                       current_state.stamp ||
                   estimatorContext_->pending_visual_measurements.empty() ||
                   estimatorContext_->pending_visual_measurements.front().stamp !=
                       measurement->stamp) {
                    stale_snapshot_detected = true;
                }
                else {
                    estimatorContext_->filter_state = *propagated_filter_state;
                    estimatorContext_->pending_visual_measurements.pop_front();
                    remaining_measurements =
                        estimatorContext_->pending_visual_measurements.size();
                    handoff_stored = true;
                }
            }

            if(stale_snapshot_detected) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Discarded prepared visual handoff because estimator state or queue advanced concurrently"
                );
                return;
            }
            if(handoff_stored) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Visual measurement handoff ready: stamp=%.9f s, position_world_imu=[%.9f, %.9f, %.9f] m, world_from_imu_xyzw=[%.9f, %.9f, %.9f, %.9f], quaternion_norm=%.9f, pending=%zu",
                    measurement->stamp.seconds(),
                    measurement->position_world_imu.x(),
                    measurement->position_world_imu.y(),
                    measurement->position_world_imu.z(),
                    measurement->world_from_imu.x(),
                    measurement->world_from_imu.y(),
                    measurement->world_from_imu.z(),
                    measurement->world_from_imu.w(),
                    measurement->world_from_imu.norm(),
                    remaining_measurements
                );
            }
        }
    }
}  // namespace vio_node
