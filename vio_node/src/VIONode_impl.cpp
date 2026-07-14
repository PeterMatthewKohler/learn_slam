#include <vio_node/VIONode_impl.hpp>
#include <utility>

namespace vio_node {
    VIONode::VIONode(const rclcpp::NodeOptions& options) : Node("vio_node", options)
    {
        // Initialize parameters
        initParameters();
        // Init TF
        initTF();
        // Initialize publishers and subscribers
        initPubSubs();
    }

    void VIONode::initPubSubs()
    {
        // Initialize publishers and subscribers
        // Publishers
        vioOdomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(vioPubTopicName_, 10);
        if(publishDebugStereoFeatures_){debugStereoFeaturePub_ = 
                                            this->create_publisher<sensor_msgs::msg::Image>("debug/StereoFeatures", 10);}
        // Subscribers
        auto imageQOS = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imuSubTopicName_, 10, std::bind(&VIONode::imuCallback, this, std::placeholders::_1));
        // Synchronize stereo camera image subscribers
        leftImgSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, leftImgSubTopicName_, imageQOS.get_rmw_qos_profile()
        );
        rightImgSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, rightImgSubTopicName_, imageQOS.get_rmw_qos_profile()
        );
        sync_ = std::make_shared<message_filters::Synchronizer<StereoSyncPolicy>>(
            StereoSyncPolicy(10), *leftImgSub_, *rightImgSub_
        );
        sync_->registerCallback(std::bind(&VIONode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.01));
        // Camera info
        leftCameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            leftCameraInfoSubTopicName_, 10, std::bind(&VIONode::leftCameraInfoCallback, this, std::placeholders::_1));
        rightCameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            rightCameraInfoSubTopicName_, 10, std::bind(&VIONode::rightCameraInfoCallback, this, std::placeholders::_1));
    }

    void VIONode::initTF()
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_,
            this,
            false   // Node's executor services /tf and /tf_static, doesn't need its own thread
        );

        extrinsicsInitTimer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&VIONode::tryInitializeExtrinsics, this)
        );
    }

    void VIONode::tryInitializeExtrinsics()
    {
        if(extrinsicsInitialized_){return;}

        try {
            auto imu_from_left = tf_buffer_->lookupTransform(
                imuFrameID_, leftCameraFrameID_, tf2::TimePointZero);
            auto imu_from_right = tf_buffer_->lookupTransform(
                imuFrameID_, rightCameraFrameID_, tf2::TimePointZero);
            auto imu_from_body = tf_buffer_->lookupTransform(
                imuFrameID_, bodyFrameID_, tf2::TimePointZero);
            auto left_from_right = tf_buffer_->lookupTransform(
                leftCameraFrameID_, rightCameraFrameID_, tf2::TimePointZero);
            // Validate transforms
            const bool transforms_valid =
                validateTransform(imu_from_left, imuFrameID_, leftCameraFrameID_) &&
                validateTransform(imu_from_right, imuFrameID_, rightCameraFrameID_) &&
                validateTransform(imu_from_body, imuFrameID_, bodyFrameID_) &&
                validateTransform(left_from_right, leftCameraFrameID_, rightCameraFrameID_) &&
                validateStereoTransform(left_from_right);
            if(!transforms_valid){return;}
            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                imuFromLeftCamera_ = std::move(imu_from_left);
                imuFromRightCamera_ = std::move(imu_from_right);
                imuFromBody_ = std::move(imu_from_body);
                tfStereoBaselineM_ = left_from_right.transform.translation.x;
                extrinsicsInitialized_ = true;
            }

            extrinsicsInitTimer_->cancel(); // Only need this once
            RCLCPP_INFO(get_logger(), "Cached VIO sensor extrinsics");
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting For VIO Sensor Extrinsics: %s", ex.what());
        }
    }

    bool VIONode::validateFrameID(const std::string& actual, const std::string& expected,
                                  const std::string& sensor_name)
    {
        if(actual != expected){
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Mismatch between %s sensor: actual name(%s) and expected name(%s)",
                sensor_name.c_str(), actual.c_str(), expected.c_str());
            return false;
        }
        return true;
    }

    bool VIONode::validateTransform(const geometry_msgs::msg::TransformStamped& transform,
                                    const std::string& expected_target,
                                    const std::string& expected_source)
    {
        // Validate Frame IDs
        if(transform.child_frame_id != expected_source) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Mismatch between actual source frame(%s) and expected source frame(%s)",
                transform.child_frame_id.c_str(), expected_source.c_str());
                return false;
        }
        if(transform.header.frame_id != expected_target) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Mismatch between actual target frame(%s) and expected target frame(%s)",
                transform.header.frame_id.c_str(), expected_target.c_str());
                return false;
        }
        const auto& translation = transform.transform.translation;
        const auto& rotation = transform.transform.rotation;

        // Check every translation and quaternion component.
        const bool components_are_finite =
            std::isfinite(translation.x) &&
            std::isfinite(translation.y) &&
            std::isfinite(translation.z) &&
            std::isfinite(rotation.x) &&
            std::isfinite(rotation.y) &&
            std::isfinite(rotation.z) &&
            std::isfinite(rotation.w);

        if (!components_are_finite) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Transform components from parent '%s' to child '%s' are not finite",
                transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
            return false;
        }
        // Check quaternion norm within tolerance value of 1.0
        double quat_norm_tolerance = 1e-3;
        const double quaternion_norm = std::sqrt(
            rotation.x * rotation.x +
            rotation.y * rotation.y +
            rotation.z * rotation.z +
            rotation.w * rotation.w);
        bool quatTolGood = std::abs(quaternion_norm - 1.0) <= quat_norm_tolerance;
        if(!quatTolGood) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Quaternion norm outside of tolerance range.");
        }
        return quatTolGood;
    }

    bool VIONode::validateStereoTransform(const geometry_msgs::msg::TransformStamped& left_from_right)
    {
        // Validate translation and rotation
        auto translation = left_from_right.transform.translation;
        auto rotation = left_from_right.transform.rotation;
        constexpr double tolerance = 1e-3;
        // Check rotation is approx. identity
        auto checkIdentity = [] (geometry_msgs::msg::Quaternion q,
                                 double tolerance) {
            const double norm = std::sqrt(
                q.x * q.x +
                q.y * q.y +
                q.z * q.z +
                q.w * q.w);
            if (!std::isfinite(norm) || norm < tolerance) {return false;}
            // q and -q represent the same rotation.
            const double normalized_abs_w =
                std::clamp(std::abs(q.w) / norm, 0.0, 1.0);
            const double rotation_angle =
                2.0 * std::acos(normalized_abs_w);

            return rotation_angle <= tolerance;
        };
        if(translation.x <= 0 ||
           std::abs(translation.y) > tolerance ||
           std::abs(translation.z) > tolerance ||
           !checkIdentity(rotation, tolerance)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Translation failed sanity check and/or quaternion norm outside of tolerance range for transform from parent '%s' to child '%s'.",
                left_from_right.header.frame_id.c_str(), left_from_right.child_frame_id.c_str());
            return false;
        }

        return true;
    }

    void VIONode::initParameters()
    {
        this->declare_parameter("imu_sub_topic_name", "/chassis/imu");
        imuSubTopicName_ = this->get_parameter("imu_sub_topic_name").as_string();

        this->declare_parameter("left_img_sub_topic_name", "/front_stereo_camera/left/image_raw");
        leftImgSubTopicName_ = this->get_parameter("left_img_sub_topic_name").as_string();

        this->declare_parameter("left_camera_info_sub_topic_name", "/front_stereo_camera/left/camera_info");
        leftCameraInfoSubTopicName_ = this->get_parameter("left_camera_info_sub_topic_name").as_string();

        this->declare_parameter("right_img_sub_topic_name", "/front_stereo_camera/right/image_raw");
        rightImgSubTopicName_ = this->get_parameter("right_img_sub_topic_name").as_string();

        this->declare_parameter("right_camera_info_sub_topic_name", "/front_stereo_camera/right/camera_info");
        rightCameraInfoSubTopicName_ = this->get_parameter("right_camera_info_sub_topic_name").as_string();

        this->declare_parameter("vio_pub_topic_name", "/vio/odom");
        vioPubTopicName_ = this->get_parameter("vio_pub_topic_name").as_string();

        this->declare_parameter("publish_debug_stereo_features", false);
        publishDebugStereoFeatures_ = this->get_parameter("publish_debug_stereo_features").as_bool();

        this->declare_parameter("world_frame_id", "vio_odom");
        worldFrameID_ = this->get_parameter("world_frame_id").as_string();

        this->declare_parameter("body_frame_id", "base_link");
        bodyFrameID_ = this->get_parameter("body_frame_id").as_string();

        this->declare_parameter("imu_frame_id", "chassis_imu");
        imuFrameID_ = this->get_parameter("imu_frame_id").as_string();

        this->declare_parameter("left_camera_frame_id", "front_stereo_camera_left_rgb");
        leftCameraFrameID_ = this->get_parameter("left_camera_frame_id").as_string();

        this->declare_parameter("right_camera_frame_id", "front_stereo_camera_right_rgb");
        rightCameraFrameID_ = this->get_parameter("right_camera_frame_id").as_string();

        this->declare_parameter("publish_tf", false);
        publishTF_ = this->get_parameter("publish_tf").as_bool();

        this->declare_parameter("camera_imu_time_offset_sec", 0.0);
        cameraIMUTimeOffsetS_ = this->get_parameter("camera_imu_time_offset_sec").as_double();
        if (!std::isfinite(cameraIMUTimeOffsetS_)) {throw std::invalid_argument("camera_imu_time_offset_sec must be finite");}

    }

    void VIONode::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        // Reject incorrect frame IDs
        if(!validateFrameID(msg->header.frame_id, imuFrameID_, "IMU")){return;}
        rclcpp::Time imu_stamp(msg->header.stamp);
        std::lock_guard<std::mutex> lock(dataMutex_);
        if(lastImuStamp_ && imu_stamp <= lastImuStamp_.value()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "IMU message rejected, current timestamp older than latest accepted. Current=%.9f, Latest=%.9f",
                imu_stamp.seconds(), lastImuStamp_.value().seconds());
            return;
        }
        lastImuStamp_.emplace(imu_stamp);
        currentImu_ = *msg;
    }

    void VIONode::leftCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
        // Validate frame ID
        if(!validateFrameID(msg->header.frame_id, leftCameraFrameID_, "LeftCamInfo")){return;}
        // These should never change/deviate at runtime
        std::lock_guard<std::mutex> lock(dataMutex_);
        if(!currentLeftCamInfo_.has_value()){
            currentLeftCamInfo_.emplace(*msg);
            initializeRectification(currentLeftCamInfo_.value(), leftRectMap_);
        }

        if(!stereoCalib_.initialized &&
           currentLeftCamInfo_.has_value() && currentRightCamInfo_.has_value()){
            initializeStereoCalibration(currentLeftCamInfo_.value(), currentRightCamInfo_.value(), stereoCalib_);
            RCLCPP_INFO(get_logger(),
                        "Stereo calib: fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.4f m",
                        stereoCalib_.fx,
                        stereoCalib_.fy,
                        stereoCalib_.cx,
                        stereoCalib_.cy,
                        stereoCalib_.baseline_m);
        }
    }

    void VIONode::rightCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
        // Validate frame ID
        if(!validateFrameID(msg->header.frame_id, rightCameraFrameID_, "RightCamInfo")){return;}
        // These should never change/deviate at runtime
        std::lock_guard<std::mutex> lock(dataMutex_);
        if(!currentRightCamInfo_.has_value()){
            currentRightCamInfo_.emplace(*msg);
            initializeRectification(currentRightCamInfo_.value(), rightRectMap_);
        }

        if(!stereoCalib_.initialized &&
           currentLeftCamInfo_.has_value() && currentRightCamInfo_.has_value()){
            initializeStereoCalibration(currentLeftCamInfo_.value(), currentRightCamInfo_.value(), stereoCalib_);
            RCLCPP_INFO(get_logger(),
                        "Stereo calib: fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.4f m",
                        stereoCalib_.fx,
                        stereoCalib_.fy,
                        stereoCalib_.cx,
                        stereoCalib_.cy,
                        stereoCalib_.baseline_m);
        }
    }

    void VIONode::stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                                 const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        // Validate frame IDs
        if(!validateFrameID(left_msg->header.frame_id, leftCameraFrameID_, "LeftCam") ||
           !validateFrameID(right_msg->header.frame_id, rightCameraFrameID_, "RightCam")){return;}

        // Simple check to ensure synchronization
        const auto left_stamp = rclcpp::Time(left_msg->header.stamp);
        const auto right_stamp = rclcpp::Time(right_msg->header.stamp);
        const double stereo_dt = std::abs((left_stamp - right_stamp).seconds());
        if (stereo_dt > 0.010) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Rejecting stereo pair: dt=%.6f s exceeds 0.010 s",
                stereo_dt
            );
            return;
        }
        // Check readiness
        bool ready;
        double tf_baseline, camera_baseline;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            ready = extrinsicsInitialized_ &&
                    leftRectMap_.initialized &&
                    rightRectMap_.initialized &&
                    stereoCalib_.initialized;
            tf_baseline = tfStereoBaselineM_;
            camera_baseline = stereoCalib_.baseline_m;
        }
        if(!ready){
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Waiting for VIO calibration and extrinsics");
            return;
        }
        // Validate baselines and use result
        const bool baselines_valid = std::isfinite(tf_baseline) &&
                                     std::isfinite(camera_baseline) &&
                                     tf_baseline > 0.0 &&
                                     camera_baseline > 0.0;

        const double tolerance = std::max(0.001, 0.01 * tf_baseline);

        if (!baselines_valid ||
            std::abs(tf_baseline - camera_baseline) > tolerance) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "TF Baseline(%.3f) and Camera Baseline(%.3f) difference outside of tolerance(%.3f) value",
                tf_baseline, camera_baseline, tolerance);
            return;
        }
        // Calculate midpoint of left & right images
        const int64_t midpoint_ns =
            left_stamp.nanoseconds() +
            (right_stamp.nanoseconds() - left_stamp.nanoseconds()) / 2;
        // Construct corrected timestamp
        const rclcpp::Time camera_stamp(midpoint_ns, left_stamp.get_clock_type());
        const rclcpp::Time visual_stamp = camera_stamp + rclcpp::Duration::from_seconds(cameraIMUTimeOffsetS_);
        // Reject if our visual stamp is before last accepted visual stamp
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            if(lastVisualStamp_ && visual_stamp <= lastVisualStamp_.value()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Corrected stereo timestamp rejected, not newer than latest accepted. Current=%.9f, Latest=%.9f",
                    visual_stamp.seconds(), lastVisualStamp_.value().seconds());
                return;
            }
            lastVisualStamp_.emplace(visual_stamp);
        }

        // Rectify the stereo pair of images
        cv_bridge::CvImageConstPtr left_cv;
        cv_bridge::CvImageConstPtr right_cv;
        try {
            left_cv = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::MONO8);
            right_cv = cv_bridge::toCvShare(right_msg, sensor_msgs::image_encodings::MONO8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat left_rectified;
        cv::Mat right_rectified;
        cv::remap(
            left_cv->image,
            left_rectified,
            leftRectMap_.map1,
            leftRectMap_.map2,
            cv::INTER_LINEAR
        );
        cv::remap(
            right_cv->image,
            right_rectified,
            rightRectMap_.map1,
            rightRectMap_.map2,
            cv::INTER_LINEAR
        );
        processRectifiedStereo(left_rectified, right_rectified, visual_stamp);

    }

}   // namespace vio_node
