#include <vio_node/VIONode_impl.hpp>
#include <filesystem>
#include <iostream>
#include <std_msgs/msg/header.hpp>
#include <utility>

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

    std::vector<cv::Point2f> VIONode::detectLeftFeatures(const cv::Mat& leftRectMap,
                                                         const cv::Mat& mask,
                                                         int max_corners)
    {
        std::vector<cv::Point2f> points;

        const double quality_level = 0.01;
        const double min_distance = 15.0;
        const int block_size = 7;
        const bool use_harris = false;
        const double k = 0.04;
        // Really expensive, how can I minimize usage here?
        cv::goodFeaturesToTrack(
            leftRectMap,    // Input grayscale img
            points,         // Output vector of corners
            max_corners,    // Max number of corners to return
            quality_level,  // Characterizes min accepted quality of image corners
            min_distance,   // Min possible euclidean distance between returned corners
            mask,           // Optional: Region of interest - empty to use whole input img
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

        const int min_features = 300;
        const int max_features = 500;
        const int detect_every_n_frames = 5;

        if(prev_left_rectified_.empty() || tracked_features_.empty()) {
            tracked_features_.clear();

            addNewTrackedFeatures(leftRectImg, rightRectImg, stereoCalib, max_features);
        }
        else {
            // Always preserve and track existing features
            trackExistingFeaturesTemporal(prev_left_rectified_, leftRectImg);
            // Update stereo depth for surviving features
            updateStereoDepthForTrackedFeatures(leftRectImg, rightRectImg, stereoCalib);
            
            const bool should_add_new = tracked_features_.size() < static_cast<std::size_t>(min_features) ||
                                        frame_idx_ % detect_every_n_frames == 0;
            if(should_add_new) {
                const int num_to_add = std::max(std::size_t(0), (max_features - static_cast<std::size_t>(tracked_features_.size())));
                if(num_to_add > 0) {
                    addNewTrackedFeatures(leftRectImg, rightRectImg, stereoCalib, num_to_add);
                }
            }
        }
        // Debug publishing
        std::vector<StereoFeature> features;
        features.reserve(tracked_features_.size());
        for(const auto& tracked_feature : tracked_features_) {
            StereoFeature feature;
            feature.id = tracked_feature.id;
            feature.px_left = tracked_feature.px_left_curr;
            feature.px_right = tracked_feature.px_right_curr;
            feature.disparity = tracked_feature.disparity;
            feature.depth_m = tracked_feature.depth_m;
            feature.point_left_cam = tracked_feature.point_left_cam_curr;
            features.push_back(feature);
        }

        static std::size_t debug_frame_count = 0;
        debug_frame_count++;
        if(publishDebugStereoFeatures_ && debugStereoFeaturePub_ && debug_frame_count % 10 == 0){   // Throttle publishing
            cv::Mat debug_img = makeStereoDebugImage(leftRectImg, rightRectImg, features);
            auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();
            debug_msg->header.stamp = stamp;
            debug_msg->header.frame_id = leftCameraFrameID_;
            debugStereoFeaturePub_->publish(*debug_msg);
        }

        prev_left_rectified_ = leftRectImg.clone();
        frame_idx_++;
    }

    void VIONode::trackExistingFeaturesTemporal(const cv::Mat& prev_left, const cv::Mat& curr_left)
    {
        // Track existing features frame to frame
        std::vector<cv::Point2f> prev_points;
        prev_points.reserve(tracked_features_.size());

        for (const auto& f : tracked_features_) {prev_points.push_back(f.px_left_curr);}
        if(prev_points.empty()) {return;}

        std::vector<cv::Point2f> curr_points;
        std::vector<unsigned char> status;
        std::vector<float> errors;

        cv::calcOpticalFlowPyrLK(
            prev_left,
            curr_left,
            prev_points,
            curr_points,
            status,
            errors,
            cv::Size(21, 21),
            3,
            cv::TermCriteria(
                cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                30,
                0.01
            )
        );
        // Reverse check points
        std::vector<cv::Point2f> backtracked_points;
        backtracked_points.reserve(prev_points.size());
        std::vector<unsigned char> backward_status;
        std::vector<float> backward_errors;

        cv::calcOpticalFlowPyrLK(
            curr_left,
            prev_left,
            curr_points,
            backtracked_points,
            backward_status,
            backward_errors,
            cv::Size(21, 21),
            3,
            cv::TermCriteria(
                cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                30,
                0.01
            )
        );
        // Now filter
        std::vector<TrackedFeature> surviving_features;
        const float max_temporal_lk_error = 20.0f;
        const double max_temporal_fb_error_px = 1.0;

        for(std::size_t i = 0; i < tracked_features_.size(); i++) {
            if(!status[i] || !backward_status[i]){continue;}
            if(errors[i] > max_temporal_lk_error ||
               backward_errors[i] > max_temporal_lk_error){continue;}

            const double fb_error = cv::norm(prev_points[i] - backtracked_points[i]);
            if(fb_error > max_temporal_fb_error_px){continue;}

            const auto& pt = curr_points[i];
            if(pt.x < 0 || pt.x >= curr_left.cols ||
               pt.y < 0 || pt.y >= curr_left.rows){continue;}
            
            TrackedFeature f = tracked_features_[i];
            f.px_left_prev = f.px_left_curr;
            f.point_left_cam_prev = f.point_left_cam_curr;
            f.px_left_curr = pt;
            f.age++;

            surviving_features.push_back(f);
        }
        tracked_features_ = std::move(surviving_features);
    }

    void VIONode::updateStereoDepthForTrackedFeatures(const cv::Mat& curr_left, const cv::Mat& curr_right,
                                                      const StereoCalibration& calib)
    {
        // For each current-left feature, track into the current right image
        std::vector<cv::Point2f> curr_left_points;
        curr_left_points.reserve(tracked_features_.size());
        for(const auto& f : tracked_features_) {curr_left_points.push_back(f.px_left_curr);}
        if(curr_left_points.empty()) {return;}

        std::vector<StereoMatch> stereo_matches = computeStereoMatches(curr_left, curr_right,
                                                                       curr_left_points, calib);
        std::vector<TrackedFeature> depth_valid_features;
        depth_valid_features.reserve(stereo_matches.size());
        for (const auto& match : stereo_matches) {
            TrackedFeature f = tracked_features_[match.input_index];
            f.px_right_curr = match.px_right;
            f.disparity = match.disparity;
            f.depth_m = match.depth_m;
            f.point_left_cam_curr = match.point_left_cam;
            depth_valid_features.push_back(f);
        }
        tracked_features_ = std::move(depth_valid_features);
    }

    std::vector<StereoMatch> VIONode::computeStereoMatches(const cv::Mat& left_img,
                                                           const cv::Mat& right_img,
                                                           const std::vector<cv::Point2f>& left_points,
                                                           const StereoCalibration& calib) const
    {
        // For each left feature, track into the right image
        if(left_points.empty()) {return {};}

        std::vector<cv::Point2f> right_points;
        std::vector<unsigned char> stereo_status;
        std::vector<float> stereo_errors;
        cv::calcOpticalFlowPyrLK(
            left_img,
            right_img,
            left_points,
            right_points,
            stereo_status,
            stereo_errors,
            cv::Size(21, 21),
            3,
            cv::TermCriteria(
                cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                30,
                0.01
            )
        );
        // Now update depth
        std::vector<StereoMatch> stereo_matches;

        const double max_y_error_px = 2.0;
        const double min_depth_m = 0.25;
        const double max_depth_m = 30.0;
        const double min_disparity_px = calib.fx * calib.baseline_m / max_depth_m;
        const double max_disparity_px = calib.fx * calib.baseline_m / min_depth_m;
        const float max_stereo_lk_error = 20.0f;

        for(std::size_t i = 0; i < left_points.size(); i++) {
            if(!stereo_status[i]){continue;}

            const cv::Point2f& pl = left_points[i];
            const cv::Point2f& pr = right_points[i];
            // Y error check
            const double y_error = std::abs(pl.y - pr.y);
            if(y_error > max_y_error_px){continue;}
            // Disparity check
            const double disparity = static_cast<double>(pl.x - pr.x);
            if(disparity < min_disparity_px || disparity > max_disparity_px){continue;}
            // Error check
            if(stereo_errors[i] > max_stereo_lk_error){continue;}
            // Calculate depth
            const double depth = calib.fx * calib.baseline_m / disparity;
            if(depth < min_depth_m || depth > max_depth_m){continue;}

            StereoMatch match;
            match.input_index = i;
            match.px_right = pr;
            match.disparity = disparity;
            match.depth_m = depth;
            // Calculate 3D coordinates
            const double x = (static_cast<double>(pl.x) - calib.cx) * depth / calib.fx;
            const double y = (static_cast<double>(pl.y) - calib.cy) * depth / calib.fy;
            const double z = depth;

            match.point_left_cam = cv::Point3d(x, y, z);
            stereo_matches.push_back(match);
        }
        return stereo_matches;
    }

    cv::Mat VIONode::buildFeatureDetectionMask(const cv::Size& image_size,
                                               const std::vector<TrackedFeature>& existing_features) const
    {
        cv::Mat mask(image_size, CV_8UC1, cv::Scalar(255));
        const int exclusion_radius_px = 15;
        // Add mask to existing features to prevent double detection
        for(const auto& f : existing_features) {
            if(f.px_left_curr.x >= 0 && f.px_left_curr.x < image_size.width &&
               f.px_left_curr.y >= 0 && f.px_left_curr.y < image_size.height) {
                cv::circle(mask, f.px_left_curr, exclusion_radius_px, cv::Scalar(0), -1);
            }
        }
        // Avoid features too close to image borders
        const int border = 20;
        mask.rowRange(0, border).setTo(0);  // Top
        mask.rowRange(image_size.height - border, image_size.height).setTo(0);  // Bottom
        mask.colRange(0, border).setTo(0);  // Left
        mask.colRange(image_size.width - border, image_size.width).setTo(0);   // Right
        return mask;
    }

    void VIONode::addNewTrackedFeatures(const cv::Mat& left, const cv::Mat& right,
                                        const StereoCalibration& calib, int max_new_features)
    {
        if(max_new_features <= 0){return;}
        // Get our mask
        cv::Mat mask = buildFeatureDetectionMask(left.size(), tracked_features_);
        // Detect new features with our mask
        std::vector<cv::Point2f> new_left_points = detectLeftFeatures(left, mask, max_new_features);
        if(new_left_points.empty()){return;}
        // Match from left to right image using optical flow
        std::vector<StereoMatch> matches = computeStereoMatches(left, right,
                                                                new_left_points, calib);
        for (const auto& match : matches) {
            const cv::Point2f& pl = new_left_points[match.input_index];
            TrackedFeature f;
            f.id = next_feature_id_++;
            f.px_left_prev = pl;
            f.px_left_curr = pl;
            f.px_right_curr = match.px_right;
            f.disparity = match.disparity;
            f.depth_m = match.depth_m;
            f.point_left_cam_curr = match.point_left_cam;
            f.point_left_cam_prev = f.point_left_cam_curr;  // Preserve old point
            f.age = 1;

            tracked_features_.push_back(f);
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
