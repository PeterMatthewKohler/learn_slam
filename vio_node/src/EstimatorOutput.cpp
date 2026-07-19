#include <vio_node/VIONode.hpp>
#include <vio_node/ErrorStateEkf.hpp>
#include <vio_node/Validation.hpp>

#include <cmath>

namespace {
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vector)
    {
        Eigen::Matrix3d skew;
        skew <<
            0.0,        -vector.z(),  vector.y(),
            vector.z(),  0.0,        -vector.x(),
            -vector.y(), vector.x(),  0.0;
        return skew;
    }

    bool validOutputCovariance(const Eigen::Matrix<double, 6, 6>& covariance)
    {
        if(!covariance.allFinite() ||
           !covariance.isApprox(covariance.transpose(), 1e-10) ||
           (covariance.diagonal().array() < 0.0).any()) {
            return false;
        }

        const Eigen::LLT<Eigen::Matrix<double, 6, 6>> covariance_llt(
            covariance
        );
        return covariance_llt.info() == Eigen::Success;
    }
}  // namespace

namespace vio_node {
    std::optional<nav_msgs::msg::Odometry> VIONode::odometryFromFilterState(
        const FilterState& filter_state,
        const ImuMeasurement& imu_measurement,
        double imu_sample_interval_s,
        const geometry_msgs::msg::TransformStamped& imu_from_body) const
    {
        const EstimatorState& state = filter_state.nominal_state;
        if(!error_state_ekf::validateFilterState(filter_state) ||
           !validation::useSameClock(state.stamp, imu_measurement.stamp) ||
           state.stamp != imu_measurement.stamp ||
           !validation::isFinite(imu_measurement.angular_velocity) ||
           !std::isfinite(imu_sample_interval_s) ||
           imu_sample_interval_s <= 0.0) {
            return std::nullopt;
        }

        const auto body_pose = bodyPoseFromEstimatorState(
            state,
            imu_from_body
        );
        if(!body_pose) {
            return std::nullopt;
        }

        Eigen::Quaterniond imu_from_body_quaternion(
            imu_from_body.transform.rotation.w,
            imu_from_body.transform.rotation.x,
            imu_from_body.transform.rotation.y,
            imu_from_body.transform.rotation.z
        );
        if(!validation::normalizeQuaternion(imu_from_body_quaternion)) {
            return std::nullopt;
        }

        const Eigen::Vector3d translation_imu_from_body(
            imu_from_body.transform.translation.x,
            imu_from_body.transform.translation.y,
            imu_from_body.transform.translation.z
        );
        const Eigen::Matrix3d rotation_world_from_imu =
            state.world_from_imu.toRotationMatrix();
        const Eigen::Matrix3d rotation_imu_from_world =
            rotation_world_from_imu.transpose();
        const Eigen::Matrix3d rotation_imu_from_body =
            imu_from_body_quaternion.toRotationMatrix();
        const Eigen::Matrix3d rotation_body_from_imu =
            rotation_imu_from_body.transpose();

        // Remove the estimated gyroscope bias before expressing angular
        // velocity in the body frame.
        const Eigen::Vector3d angular_velocity_imu =
            imu_measurement.angular_velocity - state.gyroscope_bias;
        const Eigen::Vector3d angular_velocity_body =
            rotation_body_from_imu * angular_velocity_imu;

        // The estimator velocity is the IMU-origin velocity in W. Add the
        // omega cross lever-arm term to obtain the body-origin velocity, then
        // rotate that velocity into B as required by nav_msgs/Odometry.
        const Eigen::Vector3d velocity_imu_coordinates =
            rotation_imu_from_world * state.velocity_world_imu;
        const Eigen::Vector3d velocity_body =
            rotation_body_from_imu *
            (velocity_imu_coordinates +
             angular_velocity_imu.cross(translation_imu_from_body));
        if(!validation::isFinite(angular_velocity_body) ||
           !validation::isFinite(velocity_body)) {
            return std::nullopt;
        }

        // Map [delta_p_W, delta_theta_I, ...] into the ROS pose covariance
        // convention [p_WB, fixed-axis orientation error in W]. The lever arm
        // couples IMU orientation uncertainty into body-position uncertainty.
        Eigen::Matrix<double, 6, error_state::state_size> pose_jacobian =
            Eigen::Matrix<double, 6, error_state::state_size>::Zero();
        pose_jacobian.block<3, 3>(0, error_state::position_index) =
            Eigen::Matrix3d::Identity();
        pose_jacobian.block<3, 3>(0, error_state::orientation_index) =
            -rotation_world_from_imu *
            skewSymmetric(translation_imu_from_body);
        pose_jacobian.block<3, 3>(3, error_state::orientation_index) =
            rotation_world_from_imu;

        Eigen::Matrix<double, 6, 6> pose_covariance =
            pose_jacobian * filter_state.covariance * pose_jacobian.transpose();
        pose_covariance = 0.5 * (pose_covariance + pose_covariance.transpose());
        if(!validOutputCovariance(pose_covariance)) {
            return std::nullopt;
        }

        // Twist is expressed in B. Its uncertainty depends on the estimated
        // velocity, local orientation error, gyroscope bias, and their cross
        // covariances in the full filter covariance.
        Eigen::Matrix<double, 6, error_state::state_size> twist_jacobian =
            Eigen::Matrix<double, 6, error_state::state_size>::Zero();
        twist_jacobian.block<3, 3>(0, error_state::orientation_index) =
            rotation_body_from_imu *
            skewSymmetric(velocity_imu_coordinates);
        twist_jacobian.block<3, 3>(0, error_state::velocity_index) =
            rotation_body_from_imu * rotation_imu_from_world;
        twist_jacobian.block<3, 3>(0, error_state::gyroscope_bias_index) =
            rotation_body_from_imu *
            skewSymmetric(translation_imu_from_body);
        twist_jacobian.block<3, 3>(3, error_state::gyroscope_bias_index) =
            -rotation_body_from_imu;

        Eigen::Matrix<double, 6, 6> twist_covariance =
            twist_jacobian * filter_state.covariance *
            twist_jacobian.transpose();

        // Convert the continuous gyroscope noise density into an approximate
        // variance for one discrete IMU sample and propagate the same noise
        // through angular velocity and the body/IMU lever arm.
        const double gyroscope_noise_variance =
            imuNoiseParameters_.gyroscope_noise_density_rad_s_sqrt_hz *
            imuNoiseParameters_.gyroscope_noise_density_rad_s_sqrt_hz /
            imu_sample_interval_s;
        if(!std::isfinite(gyroscope_noise_variance) ||
           gyroscope_noise_variance < 0.0) {
            return std::nullopt;
        }

        Eigen::Matrix<double, 6, 3> gyroscope_noise_jacobian =
            Eigen::Matrix<double, 6, 3>::Zero();
        gyroscope_noise_jacobian.block<3, 3>(0, 0) =
            -rotation_body_from_imu *
            skewSymmetric(translation_imu_from_body);
        gyroscope_noise_jacobian.block<3, 3>(3, 0) =
            rotation_body_from_imu;
        twist_covariance +=
            gyroscope_noise_jacobian *
            (Eigen::Matrix3d::Identity() * gyroscope_noise_variance) *
            gyroscope_noise_jacobian.transpose();
        twist_covariance =
            0.5 * (twist_covariance + twist_covariance.transpose());
        if(!validOutputCovariance(twist_covariance)) {
            return std::nullopt;
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = state.stamp;
        odometry.header.frame_id = worldFrameID_;
        odometry.child_frame_id = bodyFrameID_;
        odometry.pose.pose = *body_pose;
        odometry.twist.twist.linear.x = velocity_body.x();
        odometry.twist.twist.linear.y = velocity_body.y();
        odometry.twist.twist.linear.z = velocity_body.z();
        odometry.twist.twist.angular.x = angular_velocity_body.x();
        odometry.twist.twist.angular.y = angular_velocity_body.y();
        odometry.twist.twist.angular.z = angular_velocity_body.z();

        for(int row = 0; row < 6; ++row) {
            for(int column = 0; column < 6; ++column) {
                const std::size_t index =
                    static_cast<std::size_t>(row * 6 + column);
                odometry.pose.covariance[index] = pose_covariance(row, column);
                odometry.twist.covariance[index] = twist_covariance(row, column);
            }
        }
        return odometry;
    }

    void VIONode::publishEstimatorOutput(
        const FilterState& filter_state,
        const std::vector<ImuMeasurement>& imu_measurements)
    {
        if(imu_measurements.size() < std::size_t(2) || !vioOdomPub_) {
            return;
        }

        const double duration_s =
            (imu_measurements.back().stamp -
             imu_measurements.front().stamp).seconds();
        const double average_sample_interval_s =
            duration_s /
            static_cast<double>(imu_measurements.size() - std::size_t(1));

        std::optional<geometry_msgs::msg::TransformStamped> imu_from_body;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            imu_from_body = imuFromBody_;
        }
        if(!imu_from_body) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Cannot publish VIO odometry before IMU-to-body extrinsics are available"
            );
            return;
        }

        const auto odometry = odometryFromFilterState(
            filter_state,
            imu_measurements.back(),
            average_sample_interval_s,
            *imu_from_body
        );
        if(!odometry) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Failed to construct valid VIO odometry output at %.9f s",
                filter_state.nominal_state.stamp.seconds()
            );
            return;
        }

        vioOdomPub_->publish(*odometry);

        if(publishTF_) {
            if(!tf_broadcaster_) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "TF publication is enabled but the transform broadcaster is unavailable"
                );
                return;
            }

            geometry_msgs::msg::TransformStamped world_from_body;
            world_from_body.header = odometry->header;
            world_from_body.child_frame_id = odometry->child_frame_id;
            world_from_body.transform.translation.x =
                odometry->pose.pose.position.x;
            world_from_body.transform.translation.y =
                odometry->pose.pose.position.y;
            world_from_body.transform.translation.z =
                odometry->pose.pose.position.z;
            world_from_body.transform.rotation =
                odometry->pose.pose.orientation;
            tf_broadcaster_->sendTransform(world_from_body);
        }
    }
}  // namespace vio_node
