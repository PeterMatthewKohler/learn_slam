#include <vio_node/VIONode_impl.hpp>
#include <filesystem>
#include <iostream>

namespace vio_node {
    // OpenCV Helper Functions
    void VIONode::initializeRectification(
        const sensor_msgs::msg::CameraInfo& info,
        RectificationData& rect)
    {
        rect.K = cameraMatrixFromInfo(info);
        rect.D = distortionFromInfo(info);
        rect.R = rectificationMatrixFromInfo(info);
        rect.P_rect_3x3 = projectionCameraMatrixFromInfo(info);

        const cv::Size image_size(
            static_cast<int>(info.width),
            static_cast<int>(info.height)
        );

        cv::initUndistortRectifyMap(
            rect.K,
            rect.D,
            rect.R,
            rect.P_rect_3x3,
            image_size,
            CV_16SC2,
            rect.map1,
            rect.map2
        );

        rect.initialized = true;
    }

    cv::Mat VIONode::cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info)
    {
        return (cv::Mat_<double>(3, 3) <<
            info.k[0], info.k[1], info.k[2],
            info.k[3], info.k[4], info.k[5],
            info.k[6], info.k[7], info.k[8]);
    }

    cv::Mat VIONode::distortionFromInfo(const sensor_msgs::msg::CameraInfo& info)
    {
        cv::Mat D(1, static_cast<int>(info.d.size()), CV_64F);

        for (size_t i = 0; i < info.d.size(); ++i) {
            D.at<double>(0, static_cast<int>(i)) = info.d[i];
        }

        return D;
    }

    cv::Mat VIONode::rectificationMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info)
    {
        return (cv::Mat_<double>(3, 3) <<
            info.r[0], info.r[1], info.r[2],
            info.r[3], info.r[4], info.r[5],
            info.r[6], info.r[7], info.r[8]);
    }

    cv::Mat VIONode::projectionCameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info)
    {
        // Use the left 3x3 block of P as the new rectified camera matrix.
        return (cv::Mat_<double>(3, 3) <<
            info.p[0], info.p[1], info.p[2],
            info.p[4], info.p[5], info.p[6],
            info.p[8], info.p[9], info.p[10]);
    }

    namespace fs = std::filesystem;

    bool VIONode::saveRectifiedImage(const cv::Mat& rectified_img,
                                     const std::string& output_dir,
                                     const std::string& filename)
    {
        if (rectified_img.empty()) {
            std::cerr << "Error: rectified image is empty\n";
            return false;
        }

        try {
            fs::create_directories(output_dir);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Failed to create directory: " << e.what() << "\n";
            return false;
        }

        fs::path output_path = fs::path(output_dir) / filename;

        bool success = cv::imwrite(output_path.string(), rectified_img);

        if (!success) {
            std::cerr << "Failed to write image to: "
                    << output_path.string() << "\n";
            return false;
        }

        std::cout << "Saved rectified image to: "
                << output_path.string() << "\n";

        return true;
    }
}   // namespace vio_node