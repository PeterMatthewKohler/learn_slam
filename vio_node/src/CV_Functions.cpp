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

    void VIONode::initializeStereoCalibration(
        const sensor_msgs::msg::CameraInfo& left_info,
        const sensor_msgs::msg::CameraInfo& right_info,
        StereoCalibration& calib)
    {
        calib.fx = left_info.p[0];
        calib.fy = left_info.p[5];
        calib.cx = left_info.p[2];
        calib.cy = left_info.p[6];

        const double right_fx = right_info.p[0];
        const double right_tx = right_info.p[3];

        calib.baseline_m = std::abs(right_tx / right_fx);

        calib.initialized = true;
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

    std::vector<cv::Point2f> VIONode::detectLeftFeatures(const cv::Mat& leftRectMap)
    {
        std::vector<cv::Point2f> points;

        const int max_corners = 1000;
        const double quality_level = 0.01;
        const double min_distance = 15.0;
        const int block_size = 7;
        const bool use_harris = false;
        const double k = 0.04;

        cv::goodFeaturesToTrack(
            leftRectMap,    // Input grayscale img
            points,         // Output vector of corners
            max_corners,    // Max number of corners to return
            quality_level,  // Characterizes min accepted quality of image corners
            min_distance,   // Min possible euclidean distance between returned corners
            cv::Mat(),      // Optional: Region of interest - empty to use whole input img
            block_size,     // Size of averaging block for computing derivative
            use_harris,     // Indicates, whether to use operator or cornerMinEigenVal()
            k               // Free parameter of Harris detector
        );

        if(!points.empty()) {
            cv::cornerSubPix(   // Improves feature locations a bit, helps w/ depth estimates
                leftRectMap,    // input grayscale img
                points,         // input (initial guesses) and output (refined coordinates)
                cv::Size(5,5),  // Half the side length of the search window
                cv::Size(-1,-1),// Half the side length of a dead zone in the middle of the search zone
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                 20,
                                0.03)   // The termination criteria for the iterative sub-pixel refinement process
            );
        }
        return points;
    }

    // Track left features onto the right image
    // Images are rectified, match should move mostly horizontally
    // Return vector of metric 3D points in rectified left camera frame
    std::vector<StereoFeature> VIONode::matchStereoFeatures(const cv::Mat& leftRectImg,
                                                            const cv::Mat& rightRectImg,
                                                            const StereoCalibration& calib)
    {
        std::vector<StereoFeature> stereo_features;

        if(!calib.initialized || calib.baseline_m <= 0.0){return stereo_features;}

        std::vector<cv::Point2f> left_points = detectLeftFeatures(leftRectImg);
        if(left_points.empty()){return stereo_features;}

        // Find the matching right points
        std::vector<cv::Point2f> right_points;
        std::vector<unsigned char> status;
        std::vector<float> errors;
        // calculates the sub-pixel coordinates of a set of feature points
        // in a new frame based on their positions in the previous frame
        cv::calcOpticalFlowPyrLK(
            leftRectImg,            // prevImg
            rightRectImg,           // nextImg
            left_points,            // prevPts
            right_points,           // nextPts
            status,                 // output status vector
            errors,                 // output vector w/ tracking error for each feature
            cv::Size(21,21),        // Window size
            3,                      // Max level 0-based maximal pyramid level number
            cv::TermCriteria(       // Specifies termination criteria
                cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                30,
                0.01
            )
        );

        int next_feature_id = 0;
        // Stereo match filters
        const double max_y_error_px = 2.0;
        const double min_depth_m = 0.25;
        const double max_depth_m = 30.0;
        const double min_disparity_px = calib.fx * calib.baseline_m / max_depth_m;
        const double max_disparity_px = calib.fx * calib.baseline_m / min_depth_m;
        const float max_lk_error = 20.0f;
        // Debug counters
        size_t tracked_count = 0;
        size_t y_ok_count = 0;
        size_t disparity_ok_count = 0;
        size_t error_ok_count = 0;
        size_t depth_ok_count = 0;

        // ---- VALIDATION BLOCK ---- TODO REMOVE
        // std::vector<cv::Point2f> left_points_back;
        // std::vector<unsigned char> status_back;
        // std::vector<float> errors_back;

        // cv::calcOpticalFlowPyrLK(
        //     rightRectImg,
        //     leftRectImg,
        //     right_points,
        //     left_points_back,
        //     status_back,
        //     errors_back,
        //     cv::Size(21, 21),
        //     3,
        //     cv::TermCriteria(
        //         cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        //         30,
        //         0.01
        //     )
        // );
        // --- END VALIDATION BLOCK ---

        for(std::size_t i = 0; i < left_points.size(); i++) {
            if (!status[i]) {continue;}
            tracked_count++;

            // Extra validation
            // const double fb_error = cv::norm(left_points[i] - left_points_back[i]);
            // if (fb_error > 1.0) {continue;}
            // End Extra validation

            const cv::Point2f& pl = left_points[i];
            const cv::Point2f& pr = right_points[i];

            const double y_error = std::abs(pl.y - pr.y);
            if (y_error > max_y_error_px) {continue;}
            y_ok_count++;

            const double disparity = static_cast<double>(pl.x - pr.x);
            if (disparity < min_disparity_px || disparity > max_disparity_px) {continue;}
            disparity_ok_count++;

            if (errors[i] > max_lk_error) {continue;}
            error_ok_count++;

            const double depth = calib.fx * calib.baseline_m / disparity;
            if (depth < min_depth_m || depth > max_depth_m) {continue;}
            depth_ok_count++;

            const double x = (static_cast<double>(pl.x) - calib.cx) * depth / calib.fx;
            const double y = (static_cast<double>(pl.y) - calib.cy) * depth / calib.fy;
            const double z = depth;
            // Build the stereo feature
            StereoFeature feat;
            feat.id = next_feature_id++;
            feat.px_left = pl;
            feat.px_right = pr;
            feat.disparity = disparity;
            feat.depth_m = depth;
            feat.point_left_cam = cv::Point3d(x, y, z);
            stereo_features.push_back(feat);
        }
        return stereo_features;
    }

    void VIONode::processRectifiedStereo(const cv::Mat& leftRectImg, const cv::Mat& rightRectImg,
                                         const rclcpp::Time& stamp)
    {
        StereoCalibration stereoCalib;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            stereoCalib = stereoCalib_;
        }
        if(!stereoCalib.initialized) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Waiting for stereo calibration..."
            );
            return;
        }

        std::vector<StereoFeature> features = matchStereoFeatures(leftRectImg, rightRectImg, stereoCalib);
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Stereo features: %zu",
            features.size()
        );
        // For now just debug, next block: store as current frame, then track features across time
        static std::size_t debug_frame_count = 0;
        debug_frame_count++;
        if(publishDebugStereoFeatures_ && debugStereoFeaturePub_ && debug_frame_count % 10 == 0){
            cv::Mat debug_img = makeStereoDebugImage(leftRectImg, rightRectImg, features);
            auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();
            debug_msg->header.stamp = stamp;
            debug_msg->header.frame_id = "front_stereo_camera_left_optical";
            debugStereoFeaturePub_->publish(*debug_msg);
        }
    }

    cv::Mat VIONode::makeStereoDebugImage(const cv::Mat& leftRectImg, const cv::Mat& rightRectImg,
                                 const std::vector<StereoFeature>& features)
    {
        cv::Mat left_bgr;
        cv::Mat right_bgr;
        cv::cvtColor(leftRectImg, left_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(rightRectImg, right_bgr, cv::COLOR_GRAY2BGR);

        cv::Mat debug;
        cv::hconcat(left_bgr, right_bgr, debug);

        const int x_offset = leftRectImg.cols;
        // Draw every 5th feature to reduce visual clutter
        for(std::size_t i = 0; i < features.size(); i += 10) {
            cv::Point2f pl = features[i].px_left;
            cv::Point2f pr = features[i].px_right;
            pr.x += static_cast<float>(x_offset);

            cv::circle(debug, pl, 3, cv::Scalar(0, 255, 0), -1);
            cv::circle(debug, pr, 3, cv::Scalar(0, 255, 0), -1);
            cv::line(debug, pl, pr, cv::Scalar(0, 255, 0), 1);
        }
        return debug;
    }
}   // namespace vio_node
