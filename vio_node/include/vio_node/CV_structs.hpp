#ifndef CV_STRUCTS_HPP
#define CV_STRUCTS_HPP

#include <opencv2/core.hpp>
#include <cstddef>
#include <vector>

namespace vio_node {
    // Helper structs
    // Stereo Tracking
    struct StereoFeature {
        int id = -1;

        cv::Point2f px_left;
        cv::Point2f px_right;

        double disparity = 0.0;
        double depth_m = 0.0;

        // 3D point in rectified left frame
        cv::Point3d point_left_cam;
    };

    struct StereoMatch {
        std::size_t input_index;

        cv::Point2f px_right;
        cv::Point3d point_left_cam;

        double disparity;
        double depth_m;
    };

    struct TrackedFeature {
        int id = -1;

        cv::Point2f px_left_prev;           // pixel at t_(k-1)
        cv::Point3d point_left_cam_prev;    // 3D point in the left camera at t_(k-1)

        cv::Point2f px_left_curr;           // pixel at t_k
        cv::Point2f px_right_curr;          // right-camera match at t_k
        cv::Point3d point_left_cam_curr;    // 3D point in the left camera at t_k

        double disparity = 0.0;
        double depth_m = 0.0;

        int age = 0;
    };

    struct StereoCalibration {
        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double baseline_m = 0.0;

        bool initialized = false;
    };

    struct RectificationData {
        cv::Mat K;
        cv::Mat D;
        cv::Mat R;
        cv::Mat P_rect_3x3;

        cv::Mat map1;
        cv::Mat map2;

        bool initialized = false;
    };

    struct VisualCorrespondence {
        int id = -1;
        int age = 0;

        cv::Point3d point_prev;
        cv::Point2f pixel_curr;
        cv::Point3d point_curr;
    };

    struct VisualPoseEstimate {
        // Pose
        cv::Matx33d rotation_curr_from_prev = cv::Matx33d::eye();
        cv::Vec3d translation_curr_from_prev{0.0, 0.0, 0.0};
        std::vector<int> inlier_indices;
        // Quality Metrics
        double inlier_ratio = 0.0;
        double reprojection_rmse_px = 0.0;
        double median_3d_error_m = 0.0;
    };
}   // namespace vio_node

#endif  // CV_STRUCTS_HPP