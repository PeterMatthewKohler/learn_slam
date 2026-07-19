#ifndef VALIDATION_HPP
#define VALIDATION_HPP

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <cmath>

namespace vio_node {
namespace validation {
    inline bool isFinite(const geometry_msgs::msg::Vector3& vector)
    {
        return std::isfinite(vector.x) &&
            std::isfinite(vector.y) &&
            std::isfinite(vector.z);
    }

    inline bool isFinite(const geometry_msgs::msg::Quaternion& quaternion)
    {
        return std::isfinite(quaternion.x) &&
            std::isfinite(quaternion.y) &&
            std::isfinite(quaternion.z) &&
            std::isfinite(quaternion.w);
    }

    inline bool isFinite(const Eigen::Vector3d& vector)
    {
        return vector.allFinite();
    }

    inline bool isFinite(const Eigen::Quaterniond& quaternion)
    {
        return quaternion.coeffs().allFinite();
    }

    inline bool isFinite(const cv::Vec3d& vector)
    {
        return std::isfinite(vector[0]) &&
            std::isfinite(vector[1]) &&
            std::isfinite(vector[2]);
    }

    inline bool isFinite(const cv::Matx33d& matrix)
    {
        for(int row = 0; row < 3; ++row) {
            for(int column = 0; column < 3; ++column) {
                if(!std::isfinite(matrix(row, column))) {
                    return false;
                }
            }
        }
        return true;
    }

    inline bool isUnitQuaternion(
        const Eigen::Quaterniond& quaternion,
        double tolerance = 1e-6)
    {
        if(!isFinite(quaternion)) {
            return false;
        }
        const double squared_norm = quaternion.squaredNorm();
        return std::isfinite(squared_norm) &&
            std::abs(squared_norm - 1.0) <= tolerance;
    }

    inline bool isUnitQuaternion(
        const geometry_msgs::msg::Quaternion& quaternion,
        double tolerance = 1e-3)
    {
        if(!isFinite(quaternion)) {
            return false;
        }
        const double norm = std::sqrt(
            quaternion.x * quaternion.x +
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z +
            quaternion.w * quaternion.w
        );
        return std::isfinite(norm) &&
            std::abs(norm - 1.0) <= tolerance;
    }

    inline bool isRotationMatrix(
        const cv::Matx33d& rotation,
        double tolerance = 1e-3)
    {
        if(!isFinite(rotation)) {
            return false;
        }

        const cv::Matx33d orthonormality = rotation.t() * rotation;
        for(int row = 0; row < 3; ++row) {
            for(int column = 0; column < 3; ++column) {
                const double expected = row == column ? 1.0 : 0.0;
                if(std::abs(orthonormality(row, column) - expected) >
                   tolerance) {
                    return false;
                }
            }
        }

        const double determinant = cv::determinant(rotation);
        return std::isfinite(determinant) &&
            std::abs(determinant - 1.0) <= tolerance;
    }

    inline bool useSameClock(
        const rclcpp::Time& first,
        const rclcpp::Time& second)
    {
        return first.get_clock_type() == second.get_clock_type();
    }

    inline Eigen::Quaterniond canonicalize(
        const Eigen::Quaterniond& quaternion)
    {
        Eigen::Quaterniond canonical = quaternion;
        if(canonical.w() < 0.0) {
            canonical.coeffs() *= -1.0;
        }
        return canonical;
    }
}  // namespace validation
}  // namespace vio_node

#endif  // VALIDATION_HPP
