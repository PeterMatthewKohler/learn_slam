#include <vio_node/VIONode_impl.hpp>
#include <std_msgs/msg/header.hpp>
#include <utility>
#include <map>
#include <queue>
#include <algorithm>

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

        const bool first_visual_frame = prev_left_rectified_.empty();
        if(first_visual_frame || tracked_features_.empty()) {
            tracked_features_.clear();
            // Init our visual pose if its our first frame
            if(first_visual_frame) {
                std::optional<geometry_msgs::msg::TransformStamped> imu_from_left;
                {
                    std::lock_guard<std::mutex> lock(dataMutex_);
                    imu_from_left = imuFromLeftCamera_;
                }
                const auto initial_camera_pose = imu_from_left
                    ? visualCameraPoseFromTransform(*imu_from_left)
                    : std::nullopt;
                if(initial_camera_pose) {
                    visualCameraPose_ = *initial_camera_pose;
                    visualPoseChainValid_ = true;
                }
                else {
                    visualCameraPose_.reset();
                    visualPoseChainValid_ = false;
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "Failed to initialize visual pose from IMU -> left camera transform"
                    );
                }
            }
            else {visualPoseChainValid_ = false;}
            // Add new tracked features
            addNewTrackedFeatures(leftRectImg, rightRectImg, stereoCalib, max_features);
        }
        else {
            // Always preserve and track existing features
            trackExistingFeaturesTemporal(prev_left_rectified_, leftRectImg);
            // Update stereo depth for surviving features
            updateStereoDepthForTrackedFeatures(leftRectImg, rightRectImg, stereoCalib);
            // Build visual correspondences
            const auto correspondences = buildVisualCorrespondences(leftRectImg.size());
            // Select spatially balanced correspondences
            const auto selected = selectSpatiallyBalancedCorrespondences(
                                    correspondences,
                                    leftRectImg.size(),
                                    3,
                                    4,
                                    200);
            // Calculate pose
            const auto pose = estimateRelativeVisualPose(selected, stereoCalib);

            if(!pose){
                RCLCPP_WARN_STREAM_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Visual PnP failed - Candidate Count: " << correspondences.size()
                    << ", Selected Count: " << selected.size());

                visualPoseChainValid_ = false;
            }
            else if(!passesVisualPoseQualityChecks(*pose)){
                RCLCPP_WARN_STREAM_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Visual pose rejected - Inlier Count: " << pose->inlier_indices.size()
                    << ", Inlier Ratio: " << pose->inlier_ratio
                    << ", Reprojection RMSE(px): " << pose->reprojection_rmse_px
                    << ", Median 3D Error(m): " << pose->median_3d_error_m);

                visualPoseChainValid_ = false;
            }
            else{   // Relative visual pose valid
                if(visualPoseChainValid_ && visualCameraPose_.has_value()) {
                    // Perform pose composition
                    const auto composed = composeVisualCameraPose(*visualCameraPose_, *pose);
                    if(!composed){
                        visualPoseChainValid_ = false;

                        RCLCPP_WARN_THROTTLE(
                            get_logger(),
                            *get_clock(),
                            1000,
                            "Visual camera pose composition failed");
                    }
                    else{
                        visualCameraPose_ = *composed;
                        std::optional<geometry_msgs::msg::TransformStamped> imu_from_left;
                        std::optional<geometry_msgs::msg::TransformStamped> imu_from_body;
                        {
                            std::lock_guard<std::mutex> lock(dataMutex_);
                            imu_from_left = imuFromLeftCamera_;
                            imu_from_body = imuFromBody_;
                        }
                        const auto body_pose = imu_from_left && imu_from_body
                            ? visualBodyPoseFromCameraPose(
                                *visualCameraPose_,
                                *imu_from_left,
                                *imu_from_body)
                            : std::nullopt;
                        const auto body_orientation = body_pose
                            ? quaternionFromRotationMatrix(
                                body_pose->rotation_world_from_body)
                            : std::nullopt;
                        if(!body_pose) {
                            RCLCPP_WARN_THROTTLE(
                                get_logger(),
                                *get_clock(),
                                1000,
                                "Failed to derive visual body pose from camera pose and extrinsics"
                            );
                        }
                        else if(!body_orientation) {
                            RCLCPP_WARN_THROTTLE(
                                get_logger(),
                                *get_clock(),
                                1000,
                                "Failed to convert visual body rotation to a quaternion"
                            );
                        }
                        else {
                            // ---- DEBUG STUFF ---
                            const double trace = pose->rotation_curr_from_prev(0, 0) + pose->rotation_curr_from_prev(1, 1) + pose->rotation_curr_from_prev(2, 2);
                            const double cos_angle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
                            const double angle_deg = std::acos(cos_angle) * 180.0 / CV_PI;

                            // Extract absolute yaw from the normalized R_W_B
                            // quaternion using the standard ZYX convention.
                            // Positive yaw is a counter-clockwise turn around
                            // the world's +Z axis under ROS REP-103 axes.
                            const double sin_yaw = 2.0 *
                                (body_orientation->w * body_orientation->z +
                                 body_orientation->x * body_orientation->y);
                            const double cos_yaw = 1.0 - 2.0 *
                                (body_orientation->y * body_orientation->y +
                                 body_orientation->z * body_orientation->z);
                            const double body_yaw_deg =
                                std::atan2(sin_yaw, cos_yaw) * 180.0 / CV_PI;
                            const double body_quaternion_norm = std::sqrt(
                                body_orientation->x * body_orientation->x +
                                body_orientation->y * body_orientation->y +
                                body_orientation->z * body_orientation->z +
                                body_orientation->w * body_orientation->w);

                            RCLCPP_INFO_STREAM_THROTTLE(
                                get_logger(),
                                *get_clock(),
                                1000,
                                "Diagnostic output - Candidate Count: " << correspondences.size()
                                << ", Selected Count: " << selected.size()
                                << ", Inlier Count: " << pose->inlier_indices.size()
                                << ", Inlier Ratio: " << pose->inlier_ratio
                                << ", Reprojection RMSE(px): " << pose->reprojection_rmse_px
                                << ", Median 3D Error(m): " << pose->median_3d_error_m
                                << ", translation_curr_from_prev: ["
                                << pose->translation_curr_from_prev[0] << ", "
                                << pose->translation_curr_from_prev[1] << ", "
                                << pose->translation_curr_from_prev[2] << "]"
                                << ", Translation Norm: "
                                << cv::norm(pose->translation_curr_from_prev)
                                << ", Rotation Magnitude(deg) : " << angle_deg
                                << ", World Camera Translation: ["
                                << visualCameraPose_->translation_world_from_camera[0] << ", "
                                << visualCameraPose_->translation_world_from_camera[1] << ", "
                                << visualCameraPose_->translation_world_from_camera[2] << "]"
                                << ", World Body Translation: ["
                                << body_pose->translation_world_from_body[0] << ", "
                                << body_pose->translation_world_from_body[1] << ", "
                                << body_pose->translation_world_from_body[2] << "]"
                                << ", World Body Quaternion(xyzw): ["
                                << body_orientation->x << ", "
                                << body_orientation->y << ", "
                                << body_orientation->z << ", "
                                << body_orientation->w << "]"
                                << ", Quaternion Norm: " << body_quaternion_norm
                                << ", World Body Yaw(deg): " << body_yaw_deg
                            );
                        }
                    }
                }
            }
            // ---------------- TODO REMOVE END -------------------------

            const bool should_add_new = tracked_features_.size() < static_cast<std::size_t>(min_features) ||
                                        frame_idx_ % detect_every_n_frames == 0;
            if(should_add_new) {
                const int num_to_add = std::max(std::size_t(0), (max_features - static_cast<std::size_t>(tracked_features_.size())));
                if(num_to_add > 0) {
                    addNewTrackedFeatures(leftRectImg, rightRectImg, stereoCalib, num_to_add);
                }
            }
        }
        // Connect IMU intervals to visual timestamps
        // Verify buffer brackets visual stamp
        std::deque<sensor_msgs::msg::Imu> buffer;
        std::optional<rclcpp::Time> last_extraction_stamp;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            buffer = imuBuffer_;
            last_extraction_stamp = lastImuExtractionStamp_;
        }

        if(buffer.size() < std::size_t(2)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Waiting for at least two buffered IMU measurements"
            );
        }
        else if(!last_extraction_stamp) {
            const rclcpp::Time newest_imu_stamp(buffer.front().header.stamp);
            const rclcpp::Time oldest_imu_stamp(buffer.back().header.stamp);
            const bool clocks_match =
                newest_imu_stamp.get_clock_type() == stamp.get_clock_type() &&
                oldest_imu_stamp.get_clock_type() == stamp.get_clock_type();
            const bool buffer_brackets_stamp = clocks_match &&
                oldest_imu_stamp <= stamp && stamp <= newest_imu_stamp;

            if(buffer_brackets_stamp) {
                std::lock_guard<std::mutex> lock(dataMutex_);
                lastImuExtractionStamp_.emplace(stamp);

                RCLCPP_INFO(
                    get_logger(),
                    "Initialized IMU extraction timestamp at %.9f s",
                    stamp.seconds()
                );
            }
            else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "IMU buffer does not bracket visual timestamp %.9fs; buffer spans %.9f to %.9fs",
                    stamp.seconds(),
                    oldest_imu_stamp.seconds(),
                    newest_imu_stamp.seconds()
                );
            }
        }
        else {
            const auto imu_measurements = extractImuMeasurements(buffer, *last_extraction_stamp, stamp);
            if(!imu_measurements) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "IMU extraction from buffer from time %.9f s to %.9f s failed",
                    last_extraction_stamp->seconds(),
                    stamp.seconds()
                );
            }
            else {
                double summed_dt_s = 0.0;
                bool timestamps_strictly_increasing = true;
                for(std::size_t i = 1; i < imu_measurements->size(); ++i) {
                    const double dt_s =
                        ((*imu_measurements)[i].stamp -
                         (*imu_measurements)[i - 1].stamp).seconds();
                    if(!std::isfinite(dt_s) || dt_s <= 0.0) {
                        timestamps_strictly_increasing = false;
                        break;
                    }
                    summed_dt_s += dt_s;
                }

                const double interval_duration_s =
                    (stamp - *last_extraction_stamp).seconds();
                constexpr double duration_tolerance_s = 1e-9;
                const bool interval_valid =
                    timestamps_strictly_increasing &&
                    imu_measurements->size() >= std::size_t(2) &&
                    imu_measurements->front().stamp == *last_extraction_stamp &&
                    imu_measurements->back().stamp == stamp &&
                    std::isfinite(summed_dt_s) &&
                    std::abs(summed_dt_s - interval_duration_s) <=
                        duration_tolerance_s;

                if(!interval_valid) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "Extracted IMU interval failed timestamp validation"
                    );
                }
                else {
                    {
                        std::lock_guard<std::mutex> lock(dataMutex_);
                        lastImuExtractionStamp_.emplace(stamp);
                    }

                    RCLCPP_INFO_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "IMU interval: samples=%zu, duration=%.9f s, first=%.9f s, last=%.9f s, summed_dt=%.9f s",
                        imu_measurements->size(),
                        interval_duration_s,
                        imu_measurements->front().stamp.seconds(),
                        imu_measurements->back().stamp.seconds(),
                        summed_dt_s
                    );
                }
            }
        }

        // Compute initialization statistics from the latest complete IMU
        // window. This is independent of whether the current visual timestamp
        // was covered, so callback ordering cannot suppress data collection.
        std::optional<ImuInitialization> imuInit;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            imuInit = imuInitialization_;
        }
        if(!imuInit && buffer.size() >= std::size_t(2)) {
            const rclcpp::Time window_end(buffer.front().header.stamp);
            const rclcpp::Time window_start = window_end - rclcpp::Duration::from_seconds(imuInitializationWindowS_);
            const auto window_measurements = extractImuMeasurements(buffer, window_start, window_end);

            if(!window_measurements) {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Waiting for a complete %.3f s IMU statistics window",
                    imuInitializationWindowS_
                );
            }
            else {
                const auto statistics =
                    computeImuWindowStatistics(*window_measurements);
                if(!statistics) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        2000,
                        "Failed to compute IMU window statistics"
                    );
                }
                else {
                    const double angular_velocity_mean_norm =
                        statistics->angular_velocity_mean.norm();
                    const double linear_acceleration_mean_norm =
                        statistics->linear_acceleration_mean.norm();
                    const bool imu_window_stationary = isImuWindowStationary(*statistics);

                    bool initialization_stored = false;
                    std::optional<ImuInitialization> initialization;
                    if(imu_window_stationary) {
                        initialization = computeImuInitialization(*statistics, window_end);
                        if(!initialization) {
                            RCLCPP_WARN_THROTTLE(
                                get_logger(),
                                *get_clock(),
                                2000,
                                "Failed to compute IMU initialization from a stationary window"
                            );
                        }
                        else {
                            std::lock_guard<std::mutex> lock(dataMutex_);
                            if(!imuInitialization_) {
                                imuInitialization_ = *initialization;
                                initialization_stored = true;
                            }
                        }
                    }

                    if(initialization_stored) {
                        const Eigen::Vector3d aligned_specific_force =
                            initialization->world_from_imu *
                            statistics->linear_acceleration_mean;

                        RCLCPP_INFO(
                            get_logger(),
                            "IMU initialization accepted at %.9f s: gyro_bias=[%.9f, %.9f, %.9f] rad/s, accel_bias=[%.9f, %.9f, %.9f] m/s^2, world_from_imu_xyzw=[%.9f, %.9f, %.9f, %.9f], gravity_world=[%.9f, %.9f, %.9f] m/s^2, aligned_specific_force=[%.9f, %.9f, %.9f] m/s^2",
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
                            aligned_specific_force.z()
                        );
                    }

                    RCLCPP_INFO_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        2000,
                        "IMU window stats: samples=%zu, duration=%.6f s, gyro_mean=[%.9f, %.9f, %.9f] rad/s, gyro_stddev=[%.9f, %.9f, %.9f] rad/s, gyro_mean_norm=%.9f rad/s, accel_mean=[%.9f, %.9f, %.9f] m/s^2, accel_stddev=[%.9f, %.9f, %.9f] m/s^2, accel_mean_norm=%.9f m/s^2, stationary=%s",
                        statistics->sample_count,
                        statistics->duration_s,
                        statistics->angular_velocity_mean.x(),
                        statistics->angular_velocity_mean.y(),
                        statistics->angular_velocity_mean.z(),
                        statistics->angular_velocity_stddev.x(),
                        statistics->angular_velocity_stddev.y(),
                        statistics->angular_velocity_stddev.z(),
                        angular_velocity_mean_norm,
                        statistics->linear_acceleration_mean.x(),
                        statistics->linear_acceleration_mean.y(),
                        statistics->linear_acceleration_mean.z(),
                        statistics->linear_acceleration_stddev.x(),
                        statistics->linear_acceleration_stddev.y(),
                        statistics->linear_acceleration_stddev.z(),
                        linear_acceleration_mean_norm,
                        imu_window_stationary ? "true" : "false"
                    );
                }
            }
        }

        const bool should_publish_debug = publishDebugStereoFeatures_ &&
                                          debugStereoFeaturePub_ &&
                                          (frame_idx_ + 1) % 10 == 0;
        if(should_publish_debug) {
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
        // Right to left validation
        std::vector<cv::Point2f> backtracked_points;
        std::vector<unsigned char> backward_status;
        std::vector<float> backward_errors;
        cv::calcOpticalFlowPyrLK(
            right_img,
            left_img,
            right_points,
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
        // Now update depth
        std::vector<StereoMatch> stereo_matches;

        const double max_y_error_px = 2.0;
        const double min_depth_m = 0.25;
        const double max_depth_m = 30.0;
        const double min_disparity_px = calib.fx * calib.baseline_m / max_depth_m;
        const double max_disparity_px = calib.fx * calib.baseline_m / min_depth_m;
        const float max_stereo_lk_error = 20.0f;
        const double max_stereo_fb_error_px = 1.0;

        for(std::size_t i = 0; i < left_points.size(); i++) {
            if(!stereo_status[i] || !backward_status[i]){continue;}

            const cv::Point2f& pl = left_points[i];
            const cv::Point2f& pr = right_points[i];
            // Y error check
            const double y_error = std::abs(pl.y - pr.y);
            if(y_error > max_y_error_px){continue;}
            // Disparity check
            const double disparity = static_cast<double>(pl.x - pr.x);
            if(disparity < min_disparity_px || disparity > max_disparity_px){continue;}
            // Error check
            if(stereo_errors[i] > max_stereo_lk_error ||
               backward_errors[i] > max_stereo_lk_error){continue;}
            const double fb_error = cv::norm(left_points[i] - backtracked_points[i]);
            if(fb_error > max_stereo_fb_error_px){continue;}
            // Calculate depth
            const double depth = calib.fx * calib.baseline_m / disparity;
            if(depth < min_depth_m || depth > max_depth_m){continue;}
            // Explicitly verify right_points[i] is inside right img
            if (pr.x < 0 || pr.x >= right_img.cols ||
                pr.y < 0 || pr.y >= right_img.rows) {continue;}
            // Build our match
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

    std::vector<VisualCorrespondence> VIONode::buildVisualCorrespondences(const cv::Size& image_size) const
    {
        /* Iterate over tracked features and retain only tracks that satisfy
            - age >= 2
            - Current pixel coordinates are finite
            - Current pixel is inside image
            - Previous and current 3D coordinates are finite
            - Previous and current z are positive
            - depth_m is finite and positive
        */
       std::vector<VisualCorrespondence> visCorrs;
       for(const auto& feature : tracked_features_) {
            if(feature.age < 2){continue;}  // age >= 2
            if (!std::isfinite(feature.px_left_curr.x) ||
                !std::isfinite(feature.px_left_curr.y)) {continue;} // Current pixel coords are finite
            if(feature.px_left_curr.x >= image_size.width || feature.px_left_curr.x < 0 ||
               feature.px_left_curr.y >= image_size.height || feature.px_left_curr.y < 0){continue;} // Current pixel is inside image
            if(!(std::isfinite(feature.point_left_cam_prev.x) &&
                 std::isfinite(feature.point_left_cam_prev.y) &&
                 std::isfinite(feature.point_left_cam_prev.z) &&
                 std::isfinite(feature.point_left_cam_curr.x) &&
                 std::isfinite(feature.point_left_cam_curr.y) &&
                 std::isfinite(feature.point_left_cam_curr.z))){continue;} // Prev and curr 3d coords are finite
            if(!(feature.point_left_cam_prev.z > 0 &&
                 feature.point_left_cam_curr.z > 0)){continue;} // Prev and curr z are positive
            if(!std::isfinite(feature.depth_m) || feature.depth_m <= 0){continue;} // depth_m is finite and positive
            // Build our visual correspondence
            VisualCorrespondence v;
            v.id = feature.id;
            v.age = feature.age;
            v.point_prev = feature.point_left_cam_prev;
            v.pixel_curr = feature.px_left_curr;
            v.point_curr = feature.point_left_cam_curr;
            visCorrs.push_back(v);
       }
       return visCorrs;
    }

    std::vector<VisualCorrespondence> VIONode::selectSpatiallyBalancedCorrespondences(
        const std::vector<VisualCorrespondence>& correspondences,
        const cv::Size& image_size,
        int grid_rows,
        int grid_cols,
        std::size_t max_correspondences) const
    {
        // Return empty for invalid image dimensions, grid dimensions, or a zero limit
        if(!std::isfinite(image_size.width) || !std::isfinite(image_size.height) ||
           image_size.width <= 0 || image_size.height <= 0 ||
           grid_rows <= 0 || grid_cols <= 0 || max_correspondences <= 0 ||
           grid_rows > image_size.height || grid_cols > image_size.width){return {};}

        // Divide each image into grid cells
        // < Row, < Col, List of VisualCorrespondences @ (Row,Col) > >
        // Max heap sorting by age w/ id as deterministic tie breaker(lower id first)
        auto cmp = [](const VisualCorrespondence& a, const VisualCorrespondence& b) {
            if(a.age != b.age){return a.age < b.age;}   // Higher age first
            return a.id > b.id; // Lower ID first
        };
        std::map<std::size_t, std::map<std::size_t,
            std::priority_queue<VisualCorrespondence, std::vector<VisualCorrespondence>, decltype(cmp)>>> grid;
        for(const auto& c : correspondences) {
            // Get the grid location of the correspondence
            float px = c.pixel_curr.x, py = c.pixel_curr.y;
            if (!std::isfinite(px) || !std::isfinite(py) ||
                px < 0.0F || py < 0.0F ||
                px >= static_cast<float>(image_size.width) ||
                py >= static_cast<float>(image_size.height)){continue;}
            // Normalized indexing and clamp to final cell
            const std::size_t row = std::min(grid_rows - 1,
                static_cast<int>(py * grid_rows / image_size.height));
            const std::size_t col = std::min(grid_cols - 1,
                static_cast<int>(px * grid_cols / image_size.width));
            // Add to grid
            grid[row].try_emplace(col, cmp).first->second.push(c);
        }
        // Select correspondences round-robin from each non-empty cell
        std::vector<VisualCorrespondence> output;
        std::size_t count = 0;
        while (count < max_correspondences)
        {
            bool empty = true;
            for (auto& [row_index, row_map] : grid){
                for (auto& [col_index, queue] : row_map){
                    if (!queue.empty()){
                        output.push_back(queue.top());
                        queue.pop();
                        ++count;
                        empty = false;
                        if (count >= max_correspondences){break;}
                    }
                }
                if (count >= max_correspondences){break;}
            }
            if (empty){break;}
        }
        return output;
    }

    std::optional<VisualPoseEstimate> VIONode::estimateRelativeVisualPose(
        const std::vector<VisualCorrespondence>& correspondences,
        const StereoCalibration& calib) const
    {
        // Validate calibration w/ positive fx and fy
        if(!calib.initialized ||
           !std::isfinite(calib.cx) || !std::isfinite(calib.cy) ||
           !std::isfinite(calib.fx) || !std::isfinite(calib.fy) ||
           calib.fx <= 0.0 || calib.fy <= 0.0){return std::nullopt;}
        // Require atleast 6 correspondences
        const std::size_t min_size_threshold = 6;
        if(correspondences.size() < min_size_threshold){return std::nullopt;}
        // Build object points and image points
        std::vector<cv::Point3d> obj_points;
        std::vector<cv::Point2f> img_points;
        for(const auto& c : correspondences){
            obj_points.push_back(c.point_prev);
            img_points.push_back(c.pixel_curr);
        }
        // Construct rectified camera matrix from fx, fy, cx, cy
        cv::Mat K_rect = (cv::Mat_<double>(3, 3) <<
            calib.fx,   0.0,        calib.cx,
            0.0,        calib.fy,   calib.cy,
            0.0,        0.0,        1.0
        );
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        cv::Matx33d rotMat;
        std::vector<int> inliers;
        std::vector<cv::Point3d> inlier_object_points;
        std::vector<cv::Point2d> inlier_image_points;
        std::vector<cv::Point2d> projected_points;

        auto isFiniteVec3 = [](const cv::Vec3d& value) {
            return std::isfinite(value[0]) &&
                    std::isfinite(value[1]) &&
                    std::isfinite(value[2]);
        };

        try {
            bool success = cv::solvePnPRansac(
                                obj_points,             // Object points
                                img_points,             // Image points
                                K_rect,                 // Camera Matrix
                                cv::Mat(),              // Distortion coeffs, empty since image is rectified
                                rvec,                   // rvec
                                tvec,                   // tvec
                                false,                  // useExtrinsicGuess
                                100,                    // iterationCount
                                2.0f,                   // reprojectionError
                                0.99,                   // confidence
                                inliers,                // inliers
                                cv::SOLVEPNP_ITERATIVE);// flags
            if(!success || inliers.size() < min_size_threshold){return std::nullopt;}
            if (!isFiniteVec3(rvec) || !isFiniteVec3(tvec)) {return std::nullopt;}
            // Convert rvec to rotation matrix
            cv::Rodrigues(rvec, rotMat);
            // Check for non-finite values
            for (double value : rotMat.val) {
                if (!std::isfinite(value)) {
                    return std::nullopt;
                }
            }
            // Reproject each inlier point
            inlier_object_points.reserve(inliers.size());
            inlier_image_points.reserve(inliers.size());
            for(int index : inliers) {
                if (index < 0 ||
                    static_cast<std::size_t>(index) >= correspondences.size()) {
                    return std::nullopt;
                }
                const auto& correspondence =
                    correspondences[static_cast<std::size_t>(index)];

                inlier_object_points.push_back(correspondence.point_prev);
                inlier_image_points.emplace_back(
                    correspondence.pixel_curr.x,
                    correspondence.pixel_curr.y);
            }
            // Reproject points for RMSE quality calculation
            cv::projectPoints(
                inlier_object_points,   // object points
                rvec,                   // rvec
                tvec,                   // tvec
                K_rect,                 // Camera matrix
                cv::Mat(),              // No distortion since pixels are rectified
                projected_points);      // image points
            if (projected_points.size() != inlier_image_points.size() ||
                projected_points.empty()) {return std::nullopt;}
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "estimateRelativeVisualPose openCV error: " << e.what());
            return std::nullopt;
        }
        VisualPoseEstimate v;
        v.rotation_curr_from_prev = rotMat;
        v.translation_curr_from_prev = tvec;
        v.inlier_indices = inliers;
        // --- Calculate quality metrics ---
        // Inlier ratio
        v.inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(correspondences.size());
        // Reprojection RMSE
        double squared_error_sum = 0.0;
        for (std::size_t i = 0; i < projected_points.size(); ++i) {
            const double dx = projected_points[i].x - inlier_image_points[i].x;
            const double dy = projected_points[i].y - inlier_image_points[i].y;
            squared_error_sum += dx * dx + dy * dy;
        }
        const double reprojection_rmse_px = std::sqrt(squared_error_sum / projected_points.size());
        if (!std::isfinite(reprojection_rmse_px)) {return std::nullopt;}
        v.reprojection_rmse_px = reprojection_rmse_px;
        // Median 3D error
        std::vector<double> point_errors_m;
        point_errors_m.reserve(inliers.size());

        for (int index : inliers) {
            if (index < 0 ||
                static_cast<std::size_t>(index) >= correspondences.size()) {return std::nullopt;}

            const auto& correspondence = correspondences[static_cast<std::size_t>(index)];

            const cv::Vec3d point_prev(correspondence.point_prev.x,
                                       correspondence.point_prev.y,
                                       correspondence.point_prev.z);
            const cv::Vec3d point_curr(correspondence.point_curr.x,
                                       correspondence.point_curr.y,
                                       correspondence.point_curr.z);

            // Predict where the previous 3D point should be in Ck.
            const cv::Vec3d point_curr_predicted = rotMat * point_prev + tvec;
            // Calculate error between prediction and current
            const double error_m = cv::norm(point_curr_predicted - point_curr);
            // Validate
            if (!std::isfinite(error_m)) {return std::nullopt;}

            point_errors_m.push_back(error_m);
        }

        if (point_errors_m.empty()) {return std::nullopt;}
        // Sort
        std::sort(point_errors_m.begin(), point_errors_m.end());
        // Get median value from sorted array
        const std::size_t middle = point_errors_m.size() / 2;
        double median_3d_error_m = 0.0;
        if (point_errors_m.size() % 2 == 0) {
            median_3d_error_m =
                0.5 * (point_errors_m[middle - 1] +
                        point_errors_m[middle]);
        }
        else {
            median_3d_error_m = point_errors_m[middle];
        }
        // Validate
        if (!std::isfinite(median_3d_error_m)) {return std::nullopt;}
        // Store
        v.median_3d_error_m = median_3d_error_m;

        return v;
    }

    bool VIONode::passesVisualPoseQualityChecks(const VisualPoseEstimate& estimate) const
    {
        constexpr std::size_t min_inliers = 30;
        constexpr double min_inlier_ratio = 0.50;
        constexpr double max_reprojection_rmse_px = 1.0;
        constexpr double max_median_3d_error_m = 0.5;

        if(estimate.inlier_indices.size() < min_inliers){return false;}
        if(!std::isfinite(estimate.inlier_ratio) ||
           !std::isfinite(estimate.reprojection_rmse_px) ||
           !std::isfinite(estimate.median_3d_error_m)){return false;}
        if(estimate.inlier_ratio < 0.0 || estimate.inlier_ratio > 1.0 ||
           estimate.reprojection_rmse_px < 0.0 ||
           estimate.median_3d_error_m < 0.0){return false;}
        if(estimate.inlier_ratio < min_inlier_ratio ||
           estimate.reprojection_rmse_px > max_reprojection_rmse_px ||
           estimate.median_3d_error_m > max_median_3d_error_m){return false;}

        return true;
    }

    std::optional<VisualCameraPose> VIONode::composeVisualCameraPose(
        const VisualCameraPose& previous_pose,
        const VisualPoseEstimate& relative_pose) const
    {
        // Helpers for validation
        const auto isFiniteRotation = [](const cv::Matx33d& rotation) {
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    if (!std::isfinite(rotation(row, col))) {
                        return false;
                    }
                }
            }
            return true;
        };
        const auto isFiniteTranslation = [](const cv::Vec3d& translation) {
            return std::isfinite(translation[0]) &&
                std::isfinite(translation[1]) &&
                std::isfinite(translation[2]);
        };

        // Validate inputs.
        if (!isFiniteRotation(relative_pose.rotation_curr_from_prev) ||
            !isFiniteTranslation(relative_pose.translation_curr_from_prev) ||
            !isFiniteRotation(previous_pose.rotation_world_from_camera) ||
            !isFiniteTranslation(previous_pose.translation_world_from_camera)) {return std::nullopt;}

        const cv::Matx33d R_Cprev_Ck = relative_pose.rotation_curr_from_prev.t();
        const cv::Vec3d t_Cprev_Ck = -R_Cprev_Ck * relative_pose.translation_curr_from_prev;

        // Validate the inverted transform before composition.
        if (!isFiniteRotation(R_Cprev_Ck) ||
            !isFiniteTranslation(t_Cprev_Ck)) {return std::nullopt;}

        // Compose
        const cv::Matx33d R_W_Ck =
            previous_pose.rotation_world_from_camera * R_Cprev_Ck;
        const cv::Vec3d t_W_Ck =
            previous_pose.translation_world_from_camera +
            previous_pose.rotation_world_from_camera * t_Cprev_Ck;

        // Validate outputs.
        if (!isFiniteRotation(R_W_Ck) ||
            !isFiniteTranslation(t_W_Ck)) {return std::nullopt;}

        return VisualCameraPose{R_W_Ck, t_W_Ck};
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
