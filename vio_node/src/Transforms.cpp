#include <vio_node/VIONode.hpp>
#include <vio_node/Validation.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace vio_node {
    void VIONode::initTF()
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_,
            this,
            false   // The node executor services /tf and /tf_static.
        );

        extrinsicsInitTimer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&VIONode::tryInitializeExtrinsics, this)
        );
    }

    void VIONode::tryInitializeExtrinsics()
    {
        if(extrinsicsInitialized_) {
            return;
        }

        try {
            auto imu_from_left = tf_buffer_->lookupTransform(
                imuFrameID_, leftCameraFrameID_, tf2::TimePointZero);
            auto imu_from_right = tf_buffer_->lookupTransform(
                imuFrameID_, rightCameraFrameID_, tf2::TimePointZero);
            auto imu_from_body = tf_buffer_->lookupTransform(
                imuFrameID_, bodyFrameID_, tf2::TimePointZero);
            auto left_from_right = tf_buffer_->lookupTransform(
                leftCameraFrameID_, rightCameraFrameID_, tf2::TimePointZero);

            const bool transforms_valid =
                validateTransform(
                    imu_from_left,
                    imuFrameID_,
                    leftCameraFrameID_) &&
                validateTransform(
                    imu_from_right,
                    imuFrameID_,
                    rightCameraFrameID_) &&
                validateTransform(
                    imu_from_body,
                    imuFrameID_,
                    bodyFrameID_) &&
                validateTransform(
                    left_from_right,
                    leftCameraFrameID_,
                    rightCameraFrameID_) &&
                validateStereoTransform(left_from_right);
            if(!transforms_valid) {
                return;
            }

            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                imuFromLeftCamera_ = std::move(imu_from_left);
                imuFromBody_ = std::move(imu_from_body);
                tfStereoBaselineM_ = left_from_right.transform.translation.x;
                extrinsicsInitialized_ = true;
            }

            extrinsicsInitTimer_->cancel();
            RCLCPP_INFO(get_logger(), "Cached VIO sensor extrinsics");
        }
        catch(const tf2::TransformException& exception) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Waiting for VIO sensor extrinsics: %s",
                exception.what()
            );
        }
    }

    bool VIONode::validateFrameID(
        const std::string& actual,
        const std::string& expected,
        const std::string& sensor_name)
    {
        if(actual == expected) {
            return true;
        }

        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Mismatch between %s sensor: actual name(%s) and expected name(%s)",
            sensor_name.c_str(),
            actual.c_str(),
            expected.c_str()
        );
        return false;
    }

    bool VIONode::validateTransform(
        const geometry_msgs::msg::TransformStamped& transform,
        const std::string& expected_target,
        const std::string& expected_source)
    {
        if(transform.child_frame_id != expected_source) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Mismatch between actual source frame(%s) and expected source frame(%s)",
                transform.child_frame_id.c_str(),
                expected_source.c_str()
            );
            return false;
        }
        if(transform.header.frame_id != expected_target) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Mismatch between actual target frame(%s) and expected target frame(%s)",
                transform.header.frame_id.c_str(),
                expected_target.c_str()
            );
            return false;
        }

        const auto& translation = transform.transform.translation;
        const auto& rotation = transform.transform.rotation;
        if(!validation::isFinite(translation) ||
           !validation::isFinite(rotation)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Transform components from parent '%s' to child '%s' are not finite",
                transform.header.frame_id.c_str(),
                transform.child_frame_id.c_str()
            );
            return false;
        }
        if(!validation::isUnitQuaternion(rotation, 1e-3)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Quaternion norm outside tolerance for transform from '%s' to '%s'",
                transform.header.frame_id.c_str(),
                transform.child_frame_id.c_str()
            );
            return false;
        }
        return true;
    }

    bool VIONode::validateStereoTransform(
        const geometry_msgs::msg::TransformStamped& left_from_right)
    {
        constexpr double tolerance = 1e-3;
        const auto& translation = left_from_right.transform.translation;
        const auto& rotation = left_from_right.transform.rotation;

        const double quaternion_norm = std::sqrt(
            rotation.x * rotation.x +
            rotation.y * rotation.y +
            rotation.z * rotation.z +
            rotation.w * rotation.w
        );
        if(!validation::isFinite(translation) ||
           !validation::isFinite(rotation) ||
           !std::isfinite(quaternion_norm) ||
           quaternion_norm <= 1e-12) {
            return false;
        }

        const double normalized_abs_w =
            std::clamp(std::abs(rotation.w) / quaternion_norm, 0.0, 1.0);
        const double rotation_angle = 2.0 * std::acos(normalized_abs_w);
        if(translation.x <= 0.0 ||
           std::abs(translation.y) > tolerance ||
           std::abs(translation.z) > tolerance ||
           rotation_angle > tolerance) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Stereo transform from '%s' to '%s' failed horizontal-baseline sanity checks",
                left_from_right.header.frame_id.c_str(),
                left_from_right.child_frame_id.c_str()
            );
            return false;
        }
        return true;
    }

    bool VIONode::transformToOpenCV(
        const geometry_msgs::msg::TransformStamped& transform,
        cv::Matx33d& rotation_target_from_source,
        cv::Vec3d& translation_target_from_source) const
    {
        const auto& translation = transform.transform.translation;
        const auto& rotation = transform.transform.rotation;
        if(!validation::isFinite(translation) ||
           !validation::isFinite(rotation)) {
            return false;
        }

        const double quaternion_norm = std::sqrt(
            rotation.x * rotation.x +
            rotation.y * rotation.y +
            rotation.z * rotation.z +
            rotation.w * rotation.w
        );
        if(!std::isfinite(quaternion_norm) || quaternion_norm <= 1e-12) {
            return false;
        }

        tf2::Quaternion quaternion(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        );
        quaternion /= quaternion_norm;

        tf2::Matrix3x3 rotation_matrix;
        rotation_matrix.setRotation(quaternion);
        rotation_target_from_source = cv::Matx33d{
            rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]
        };
        translation_target_from_source = cv::Vec3d{
            translation.x,
            translation.y,
            translation.z
        };

        return validation::isRotationMatrix(rotation_target_from_source) &&
            validation::isFinite(translation_target_from_source);
    }

    std::optional<geometry_msgs::msg::Quaternion>
    VIONode::quaternionFromRotationMatrix(const cv::Matx33d& rotation) const
    {
        if(!validation::isRotationMatrix(rotation)) {
            return std::nullopt;
        }

        const tf2::Matrix3x3 tf_rotation(
            rotation(0, 0), rotation(0, 1), rotation(0, 2),
            rotation(1, 0), rotation(1, 1), rotation(1, 2),
            rotation(2, 0), rotation(2, 1), rotation(2, 2)
        );
        tf2::Quaternion tf_quaternion;
        tf_rotation.getRotation(tf_quaternion);

        const double quaternion_norm = std::sqrt(
            tf_quaternion.x() * tf_quaternion.x() +
            tf_quaternion.y() * tf_quaternion.y() +
            tf_quaternion.z() * tf_quaternion.z() +
            tf_quaternion.w() * tf_quaternion.w()
        );
        if(!std::isfinite(quaternion_norm) || quaternion_norm <= 1e-12) {
            return std::nullopt;
        }
        tf_quaternion /= quaternion_norm;

        geometry_msgs::msg::Quaternion output;
        output.x = tf_quaternion.x();
        output.y = tf_quaternion.y();
        output.z = tf_quaternion.z();
        output.w = tf_quaternion.w();
        if(!validation::isFinite(output)) {
            return std::nullopt;
        }
        return output;
    }
}  // namespace vio_node
