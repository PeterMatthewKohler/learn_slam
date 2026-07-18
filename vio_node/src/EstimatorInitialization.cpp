#include <vio_node/VIONode.hpp>
#include <vio_node/ErrorStateEkf.hpp>
#include <vio_node/Validation.hpp>

#include <cmath>
#include <utility>

namespace vio_node {
    void VIONode::tryInitializeEstimator(const rclcpp::Time& visual_stamp)
    {
        std::deque<sensor_msgs::msg::Imu> buffer;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            if(estimatorContext_) {
                return;
            }
            buffer = imuBuffer_;
        }
        if(buffer.size() < std::size_t(2)) {
            return;
        }

        const rclcpp::Time window_end(buffer.front().header.stamp);
        const rclcpp::Time window_start = window_end -
            rclcpp::Duration::from_seconds(imuInitializationWindowS_);
        const auto window_measurements = extractImuMeasurements(
            buffer,
            window_start,
            window_end
        );
        if(!window_measurements) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for a complete %.3f s IMU statistics window",
                imuInitializationWindowS_
            );
            return;
        }

        const auto statistics =
            computeImuWindowStatistics(*window_measurements);
        if(!statistics) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Failed to compute IMU window statistics"
            );
            return;
        }
        if(!isImuWindowStationary(*statistics)) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for stationary IMU window: samples=%zu, gyro_mean_norm=%.6f rad/s, gyro_stddev_max=%.6f rad/s, accel_stddev_max=%.6f m/s^2, gravity_error=%.6f m/s^2",
                statistics->sample_count,
                statistics->angular_velocity_mean.norm(),
                statistics->angular_velocity_stddev.maxCoeff(),
                statistics->linear_acceleration_stddev.maxCoeff(),
                std::abs(statistics->linear_acceleration_mean.norm() -
                         imuStationaryGravMagMS2_)
            );
            return;
        }

        const auto initialization =
            computeImuInitialization(*statistics, window_end);
        if(!initialization) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Failed to compute IMU initialization from a stationary window"
            );
            return;
        }

        const auto initial_state = makeInitialEstimatorState(*initialization);
        if(!initial_state) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Failed to create estimator state from IMU initialization"
            );
            return;
        }
        if(!validation::useSameClock(
               initial_state->stamp,
               initialization->stamp) ||
           initial_state->stamp != initialization->stamp) {
            RCLCPP_ERROR(
                get_logger(),
                "Initial estimator state timestamp does not match IMU initialization timestamp"
            );
            return;
        }

        const auto initial_filter_state =
            error_state_ekf::makeInitialFilterState(
                *initial_state,
                initialCovarianceParameters_
            );
        if(!initial_filter_state) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to create initial filter state and covariance"
            );
            return;
        }
        if(!validation::useSameClock(visual_stamp, initialization->stamp) ||
           !validation::useSameClock(visual_stamp, window_start) ||
           !validation::useSameClock(visual_stamp, window_end)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cannot anchor visual pose because visual and IMU initialization timestamps use different ROS clocks"
            );
            return;
        }
        if(visual_stamp < window_start || visual_stamp > window_end) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cannot anchor visual pose at %.9f s because it is outside the stationary IMU window [%.9f, %.9f] s",
                visual_stamp.seconds(),
                window_start.seconds(),
                window_end.seconds()
            );
            return;
        }

        std::optional<geometry_msgs::msg::TransformStamped>
            imu_from_left_camera;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            imu_from_left_camera = imuFromLeftCamera_;
        }
        if(!imu_from_left_camera) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cannot anchor visual pose before IMU-to-left-camera extrinsics are available"
            );
            return;
        }
        if(imu_from_left_camera->header.frame_id != imuFrameID_ ||
           imu_from_left_camera->child_frame_id != leftCameraFrameID_) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cannot anchor visual pose with transform '%s' -> '%s'; expected '%s' -> '%s'",
                imu_from_left_camera->header.frame_id.c_str(),
                imu_from_left_camera->child_frame_id.c_str(),
                imuFrameID_.c_str(),
                leftCameraFrameID_.c_str()
            );
            return;
        }

        const auto initial_camera_pose =
            visualCameraPoseFromEstimatorState(
                *initial_state,
                *imu_from_left_camera
            );
        if(!initial_camera_pose) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Failed to compute initial visual camera pose from estimator state and camera extrinsics"
            );
            return;
        }

        bool initialization_stored = false;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            if(!estimatorContext_) {
                EstimatorContext context;
                context.initialization = *initialization;
                context.filter_state = *initial_filter_state;
                estimatorContext_ = std::move(context);

                visualPoseChain_ = VisualPoseChain{
                    *initial_camera_pose,
                    visual_stamp
                };
                initialization_stored = true;
            }
        }
        if(!initialization_stored) {
            return;
        }

        const Eigen::Vector3d aligned_specific_force =
            initialization->world_from_imu *
            statistics->linear_acceleration_mean;
        RCLCPP_INFO(
            get_logger(),
            "Visual camera pose anchored at %.9f s from stationary IMU initialization at %.9f s: position_world_camera=[%.9f, %.9f, %.9f] m",
            visual_stamp.seconds(),
            initialization->stamp.seconds(),
            initial_camera_pose->translation_world_from_camera[0],
            initial_camera_pose->translation_world_from_camera[1],
            initial_camera_pose->translation_world_from_camera[2]
        );
        RCLCPP_INFO(
            get_logger(),
            "IMU initialization accepted at %.9f s: gyro_bias=[%.9f, %.9f, %.9f] rad/s, accel_bias=[%.9f, %.9f, %.9f] m/s^2, world_from_imu_xyzw=[%.9f, %.9f, %.9f, %.9f], gravity_world=[%.9f, %.9f, %.9f] m/s^2, aligned_specific_force=[%.9f, %.9f, %.9f] m/s^2, initial_position_world_imu=[%.9f, %.9f, %.9f] m, initial_velocity_world_imu=[%.9f, %.9f, %.9f] m/s, state_stamp=%.9f s, timestamps_match=true",
            initialization->stamp.seconds(),
            initialization->gyroscope_bias.x(),
            initialization->gyroscope_bias.y(),
            initialization->gyroscope_bias.z(),
            initialization->accelerometer_bias.x(),
            initialization->accelerometer_bias.y(),
            initialization->accelerometer_bias.z(),
            initialization->world_from_imu.x(),
            initialization->world_from_imu.y(),
            initialization->world_from_imu.z(),
            initialization->world_from_imu.w(),
            initialization->gravity_world.x(),
            initialization->gravity_world.y(),
            initialization->gravity_world.z(),
            aligned_specific_force.x(),
            aligned_specific_force.y(),
            aligned_specific_force.z(),
            initial_state->position_world_imu.x(),
            initial_state->position_world_imu.y(),
            initial_state->position_world_imu.z(),
            initial_state->velocity_world_imu.x(),
            initial_state->velocity_world_imu.y(),
            initial_state->velocity_world_imu.z(),
            initial_state->stamp.seconds()
        );
        RCLCPP_INFO(
            get_logger(),
            "Initial filter standard deviations: position=%.6f m, orientation=%.6f rad, velocity=%.6f m/s, gyroscope_bias=%.6f rad/s, accelerometer_bias=%.6f m/s^2",
            initialCovarianceParameters_.position_stddev_m,
            initialCovarianceParameters_.orientation_stddev_rad,
            initialCovarianceParameters_.velocity_stddev_m_s,
            initialCovarianceParameters_.gyroscope_bias_stddev_rad_s,
            initialCovarianceParameters_.accelerometer_bias_stddev_m_s2
        );
    }
}  // namespace vio_node
