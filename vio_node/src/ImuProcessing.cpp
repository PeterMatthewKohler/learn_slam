#include <vio_node/VIONode.hpp>
#include <vio_node/Validation.hpp>

#include <cmath>
#include <iterator>

namespace vio_node {
    std::optional<ImuMeasurement> VIONode::interpolateImuMeasurement(
        const sensor_msgs::msg::Imu& before,
        const sensor_msgs::msg::Imu& after,
        const rclcpp::Time& target_stamp) const
    {
        const rclcpp::Time before_stamp(before.header.stamp);
        const rclcpp::Time after_stamp(after.header.stamp);
        if(!validation::useSameClock(before_stamp, after_stamp) ||
           !validation::useSameClock(before_stamp, target_stamp) ||
           before.header.frame_id != imuFrameID_ ||
           after.header.frame_id != imuFrameID_ ||
           after_stamp <= before_stamp ||
           target_stamp < before_stamp ||
           target_stamp > after_stamp ||
           !validation::isFinite(before.angular_velocity) ||
           !validation::isFinite(before.linear_acceleration) ||
           !validation::isFinite(after.angular_velocity) ||
           !validation::isFinite(after.linear_acceleration)) {
            return std::nullopt;
        }

        const double alpha = (target_stamp - before_stamp).seconds() /
            (after_stamp - before_stamp).seconds();
        if(!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
            return std::nullopt;
        }

        const auto interpolate = [alpha](double before_value, double after_value) {
            return (1.0 - alpha) * before_value + alpha * after_value;
        };

        ImuMeasurement measurement;
        measurement.stamp = target_stamp;
        measurement.angular_velocity = Eigen::Vector3d(
            interpolate(before.angular_velocity.x, after.angular_velocity.x),
            interpolate(before.angular_velocity.y, after.angular_velocity.y),
            interpolate(before.angular_velocity.z, after.angular_velocity.z)
        );
        measurement.linear_acceleration = Eigen::Vector3d(
            interpolate(
                before.linear_acceleration.x,
                after.linear_acceleration.x),
            interpolate(
                before.linear_acceleration.y,
                after.linear_acceleration.y),
            interpolate(
                before.linear_acceleration.z,
                after.linear_acceleration.z)
        );
        if(!validation::isFinite(measurement.angular_velocity) ||
           !validation::isFinite(measurement.linear_acceleration)) {
            return std::nullopt;
        }
        return measurement;
    }

    std::optional<std::vector<ImuMeasurement>> VIONode::extractImuMeasurements(
        const std::deque<sensor_msgs::msg::Imu>& buffer,
        const rclcpp::Time& start_stamp,
        const rclcpp::Time& end_stamp) const
    {
        if(!validation::useSameClock(start_stamp, end_stamp) ||
           buffer.size() < std::size_t(2) ||
           end_stamp <= start_stamp) {
            return std::nullopt;
        }

        const rclcpp::Time oldest_stamp(buffer.back().header.stamp);
        const rclcpp::Time newest_stamp(buffer.front().header.stamp);
        if(!validation::useSameClock(oldest_stamp, start_stamp) ||
           !validation::useSameClock(newest_stamp, start_stamp) ||
           oldest_stamp > start_stamp ||
           newest_stamp < end_stamp) {
            return std::nullopt;
        }

        std::vector<ImuMeasurement> measurements;
        bool start_added = false;
        auto before = buffer.rbegin();
        auto after = std::next(before);
        for(; after != buffer.rend(); ++before, ++after) {
            const rclcpp::Time before_stamp(before->header.stamp);
            const rclcpp::Time after_stamp(after->header.stamp);
            if(!validation::useSameClock(before_stamp, start_stamp) ||
               !validation::useSameClock(after_stamp, start_stamp) ||
               after_stamp <= before_stamp) {
                return std::nullopt;
            }

            if(!start_added &&
               before_stamp <= start_stamp &&
               start_stamp <= after_stamp) {
                const auto start_measurement = interpolateImuMeasurement(
                    *before,
                    *after,
                    start_stamp
                );
                if(!start_measurement) {
                    return std::nullopt;
                }
                measurements.push_back(*start_measurement);
                start_added = true;
            }
            if(!start_added) {
                continue;
            }

            if(after_stamp > start_stamp && after_stamp < end_stamp) {
                const auto interior_measurement = interpolateImuMeasurement(
                    *before,
                    *after,
                    after_stamp
                );
                if(!interior_measurement) {
                    return std::nullopt;
                }
                measurements.push_back(*interior_measurement);
            }

            if(before_stamp <= end_stamp && end_stamp <= after_stamp) {
                const auto end_measurement = interpolateImuMeasurement(
                    *before,
                    *after,
                    end_stamp
                );
                if(!end_measurement) {
                    return std::nullopt;
                }
                measurements.push_back(*end_measurement);

                if(measurements.size() < std::size_t(2) ||
                   measurements.front().stamp != start_stamp ||
                   measurements.back().stamp != end_stamp) {
                    return std::nullopt;
                }
                return measurements;
            }
        }
        return std::nullopt;
    }

    std::optional<ImuWindowStatistics> VIONode::computeImuWindowStatistics(
        const std::vector<ImuMeasurement>& measurements) const
    {
        if(measurements.size() < std::size_t(2)) {
            return std::nullopt;
        }

        const auto clock_type = measurements.front().stamp.get_clock_type();
        Eigen::Vector3d angular_velocity_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity_m2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_acceleration_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_acceleration_m2 = Eigen::Vector3d::Zero();

        std::size_t sample_count = 0;
        for(std::size_t index = 0; index < measurements.size(); ++index) {
            const auto& measurement = measurements[index];
            if(measurement.stamp.get_clock_type() != clock_type ||
               !validation::isFinite(measurement.angular_velocity) ||
               !validation::isFinite(measurement.linear_acceleration) ||
               (index > 0 &&
                measurement.stamp <= measurements[index - 1].stamp)) {
                return std::nullopt;
            }

            ++sample_count;
            const double count = static_cast<double>(sample_count);

            const Eigen::Vector3d angular_delta =
                measurement.angular_velocity - angular_velocity_mean;
            angular_velocity_mean += angular_delta / count;
            angular_velocity_m2 += angular_delta.cwiseProduct(
                measurement.angular_velocity - angular_velocity_mean
            );

            const Eigen::Vector3d acceleration_delta =
                measurement.linear_acceleration - linear_acceleration_mean;
            linear_acceleration_mean += acceleration_delta / count;
            linear_acceleration_m2 += acceleration_delta.cwiseProduct(
                measurement.linear_acceleration - linear_acceleration_mean
            );
        }

        const double duration_s =
            (measurements.back().stamp - measurements.front().stamp).seconds();
        if(!std::isfinite(duration_s) || duration_s <= 0.0) {
            return std::nullopt;
        }

        const double variance_denominator =
            static_cast<double>(sample_count - 1);
        const Eigen::Vector3d angular_velocity_variance =
            (angular_velocity_m2 / variance_denominator).cwiseMax(0.0);
        const Eigen::Vector3d linear_acceleration_variance =
            (linear_acceleration_m2 / variance_denominator).cwiseMax(0.0);

        ImuWindowStatistics statistics;
        statistics.sample_count = sample_count;
        statistics.duration_s = duration_s;
        statistics.angular_velocity_mean = angular_velocity_mean;
        statistics.angular_velocity_stddev =
            angular_velocity_variance.cwiseSqrt();
        statistics.linear_acceleration_mean = linear_acceleration_mean;
        statistics.linear_acceleration_stddev =
            linear_acceleration_variance.cwiseSqrt();

        if(!validation::isFinite(statistics.angular_velocity_mean) ||
           !validation::isFinite(statistics.angular_velocity_stddev) ||
           !validation::isFinite(statistics.linear_acceleration_mean) ||
           !validation::isFinite(statistics.linear_acceleration_stddev)) {
            return std::nullopt;
        }
        return statistics;
    }

    bool VIONode::isImuWindowStationary(
        const ImuWindowStatistics& statistics) const
    {
        return statistics.sample_count >= imuMinSamples_ &&
            statistics.angular_velocity_mean.norm() <=
                imuStationaryMaxGyroMeanNormRadS_ &&
            statistics.angular_velocity_stddev.maxCoeff() <=
                imuStationaryMaxGyroStddevRadS_ &&
            statistics.linear_acceleration_stddev.maxCoeff() <=
                imuStationaryAccelStddevMS2_ &&
            std::abs(statistics.linear_acceleration_mean.norm() -
                     imuStationaryGravMagMS2_) <= imuStationaryGravTolMS2_;
    }

    std::optional<ImuInitialization> VIONode::computeImuInitialization(
        const ImuWindowStatistics& statistics,
        const rclcpp::Time& initialization_stamp) const
    {
        if(!isImuWindowStationary(statistics)) {
            return std::nullopt;
        }

        ImuInitialization initialization;
        initialization.stamp = initialization_stamp;
        initialization.gyroscope_bias = statistics.angular_velocity_mean;
        // One stationary orientation cannot separate accelerometer bias from tilt.
        initialization.accelerometer_bias = Eigen::Vector3d::Zero();

        const Eigen::Vector3d measured_up =
            statistics.linear_acceleration_mean.normalized();
        const double roll = std::atan2(measured_up.y(), measured_up.z());
        const double pitch = std::atan2(
            -measured_up.x(),
            std::hypot(measured_up.y(), measured_up.z())
        );
        Eigen::Quaterniond world_from_imu =
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        const double squared_norm = world_from_imu.squaredNorm();
        if(!validation::isFinite(world_from_imu) ||
           !std::isfinite(squared_norm) ||
           squared_norm < 1e-12) {
            return std::nullopt;
        }
        world_from_imu.normalize();
        if(!validation::isUnitQuaternion(world_from_imu)) {
            return std::nullopt;
        }

        initialization.world_from_imu = world_from_imu;
        initialization.gravity_world = Eigen::Vector3d(
            0.0,
            0.0,
            -imuStationaryGravMagMS2_
        );
        return initialization;
    }
}  // namespace vio_node
