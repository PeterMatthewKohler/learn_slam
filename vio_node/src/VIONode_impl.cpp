#include <vio_node/VIONode_impl.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iterator>
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
            imuSubTopicName_, 200, std::bind(&VIONode::imuCallback, this, std::placeholders::_1));
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

    bool VIONode::transformToOpenCV(
        const geometry_msgs::msg::TransformStamped& transform,
        cv::Matx33d& rotation_target_from_source,
        cv::Vec3d& translation_target_from_source) const
    {
        // Validate all translation and quaternion components
        const auto& translation = transform.transform.translation;
        const auto& rotation = transform.transform.rotation;
        if(!std::isfinite(translation.x) ||
           !std::isfinite(translation.y) ||
           !std::isfinite(translation.z) ||
           !std::isfinite(rotation.x) ||
           !std::isfinite(rotation.y) ||
           !std::isfinite(rotation.z) ||
           !std::isfinite(rotation.w)){return false;}
        // Normalize
        const double quaternion_norm = std::sqrt(
            rotation.x * rotation.x +
            rotation.y * rotation.y +
            rotation.z * rotation.z +
            rotation.w * rotation.w);

        if(!std::isfinite(quaternion_norm) ||
            quaternion_norm <= 1e-12) {return false;}
        tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
        quat /= quaternion_norm;

        tf2::Matrix3x3 rotMat;
        rotMat.setRotation(quat);
        // Copy into cv Matrix
        rotation_target_from_source = cv::Matx33d{
            rotMat[0][0], rotMat[0][1], rotMat[0][2],
            rotMat[1][0], rotMat[1][1], rotMat[1][2],
            rotMat[2][0], rotMat[2][1], rotMat[2][2]};
        translation_target_from_source = cv::Vec3d{
            translation.x, translation.y, translation.z};
        // Validate output
        bool rotation_is_finite = true;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                rotation_is_finite &=
                    std::isfinite(rotation_target_from_source(row, col));
            }
        }
        const bool translation_is_finite =
            std::isfinite(translation_target_from_source[0]) &&
            std::isfinite(translation_target_from_source[1]) &&
            std::isfinite(translation_target_from_source[2]);

        return rotation_is_finite && translation_is_finite;
    }

    std::optional<VisualCameraPose> VIONode::visualCameraPoseFromTransform(
        const geometry_msgs::msg::TransformStamped& transform) const
    {
        cv::Matx33d rotation_world_from_camera;
        cv::Vec3d translation_world_from_camera;
        if(!transformToOpenCV(
            transform,
            rotation_world_from_camera,
            translation_world_from_camera)){return std::nullopt;}

        return VisualCameraPose{
            rotation_world_from_camera,
            translation_world_from_camera};
    }

    std::optional<VisualBodyPose> VIONode::visualBodyPoseFromCameraPose(
        const VisualCameraPose& world_from_camera,
        const geometry_msgs::msg::TransformStamped& imu_from_camera,
        const geometry_msgs::msg::TransformStamped& imu_from_body) const
    {
        const auto isFiniteRotation = [](const cv::Matx33d& rotation) {
            for(int row = 0; row < 3; ++row) {
                for(int col = 0; col < 3; ++col) {
                    if(!std::isfinite(rotation(row, col))){return false;}
                }
            }
            return true;
        };
        const auto isFiniteTranslation = [](const cv::Vec3d& translation) {
            return std::isfinite(translation[0]) &&
                   std::isfinite(translation[1]) &&
                   std::isfinite(translation[2]);
        };

        if(!isFiniteRotation(world_from_camera.rotation_world_from_camera) ||
           !isFiniteTranslation(world_from_camera.translation_world_from_camera)){
            return std::nullopt;
        }

        cv::Matx33d R_I_CL;
        cv::Vec3d t_I_CL;
        cv::Matx33d R_I_B;
        cv::Vec3d t_I_B;
        if(!transformToOpenCV(imu_from_camera, R_I_CL, t_I_CL) ||
           !transformToOpenCV(imu_from_body, R_I_B, t_I_B)){
            return std::nullopt;
        }

        // T_W_I = T_W_CL * inverse(T_I_CL)
        const cv::Matx33d R_CL_I = R_I_CL.t();
        const cv::Vec3d t_CL_I = -(R_CL_I * t_I_CL);
        const cv::Matx33d R_W_I =
            world_from_camera.rotation_world_from_camera * R_CL_I;
        const cv::Vec3d t_W_I =
            world_from_camera.translation_world_from_camera +
            world_from_camera.rotation_world_from_camera * t_CL_I;

        // T_W_B = T_W_I * T_I_B
        const cv::Matx33d R_W_B = R_W_I * R_I_B;
        const cv::Vec3d t_W_B = t_W_I + R_W_I * t_I_B;

        if(!isFiniteRotation(R_W_B) ||
           !isFiniteTranslation(t_W_B)){return std::nullopt;}

        return VisualBodyPose{R_W_B, t_W_B};
    }

    std::optional<geometry_msgs::msg::Quaternion> VIONode::quaternionFromRotationMatrix(
        const cv::Matx33d& rotation) const
    {
        constexpr double rotation_tolerance = 1e-3;

        // A valid rotation matrix can only contain finite values. Checking this
        // before doing matrix multiplication also prevents NaNs from hiding in
        // the orthonormality and determinant checks below.
        for(int row = 0; row < 3; ++row) {
            for(int col = 0; col < 3; ++col) {
                if(!std::isfinite(rotation(row, col))){return std::nullopt;}
            }
        }

        // The columns of a rotation matrix are mutually perpendicular unit
        // vectors. Therefore R^T * R must be the identity matrix. This catches
        // scaling and shear before passing the matrix into tf2.
        const cv::Matx33d rotation_transpose_times_rotation =
            rotation.t() * rotation;
        for(int row = 0; row < 3; ++row) {
            for(int col = 0; col < 3; ++col) {
                const double expected = row == col ? 1.0 : 0.0;
                if(std::abs(rotation_transpose_times_rotation(row, col) - expected) >
                   rotation_tolerance){return std::nullopt;}
            }
        }

        // Orthonormal matrices can describe either a rotation or a reflection.
        // A proper 3D rotation has determinant +1, while a reflection has -1.
        const double determinant =
            rotation(0, 0) * (rotation(1, 1) * rotation(2, 2) -
                              rotation(1, 2) * rotation(2, 1)) -
            rotation(0, 1) * (rotation(1, 0) * rotation(2, 2) -
                              rotation(1, 2) * rotation(2, 0)) +
            rotation(0, 2) * (rotation(1, 0) * rotation(2, 1) -
                              rotation(1, 1) * rotation(2, 0));
        if(!std::isfinite(determinant) ||
           std::abs(determinant - 1.0) > rotation_tolerance){return std::nullopt;}

        // tf2 uses the same row/column convention here: this matrix represents
        // R_W_B, which maps vectors from the body frame into the world frame.
        const tf2::Matrix3x3 tf_rotation(
            rotation(0, 0), rotation(0, 1), rotation(0, 2),
            rotation(1, 0), rotation(1, 1), rotation(1, 2),
            rotation(2, 0), rotation(2, 1), rotation(2, 2));
        tf2::Quaternion tf_quaternion;
        tf_rotation.getRotation(tf_quaternion);

        // Matrix-to-quaternion conversion should already return a unit
        // quaternion. Normalize it explicitly so downstream ROS messages do
        // not accumulate small floating-point errors from pose composition.
        const double quaternion_norm = std::sqrt(
            tf_quaternion.x() * tf_quaternion.x() +
            tf_quaternion.y() * tf_quaternion.y() +
            tf_quaternion.z() * tf_quaternion.z() +
            tf_quaternion.w() * tf_quaternion.w());
        if(!std::isfinite(quaternion_norm) ||
           quaternion_norm <= 1e-12){return std::nullopt;}
        tf_quaternion /= quaternion_norm;

        geometry_msgs::msg::Quaternion output;
        output.x = tf_quaternion.x();
        output.y = tf_quaternion.y();
        output.z = tf_quaternion.z();
        output.w = tf_quaternion.w();

        // Keep this final validation at the ROS-message boundary. It ensures a
        // future caller can safely publish every quaternion returned here.
        if(!std::isfinite(output.x) ||
           !std::isfinite(output.y) ||
           !std::isfinite(output.z) ||
           !std::isfinite(output.w)){return std::nullopt;}

        return output;
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

        this->declare_parameter("imu_msg_gap_threshold_s", 0.05);
        imuMsgGapThresholdS_ = this->get_parameter("imu_msg_gap_threshold_s").as_double();
        if (!std::isfinite(imuMsgGapThresholdS_) ||
            imuMsgGapThresholdS_ <= 0) {throw std::invalid_argument("imu_msg_gap_threshold_s must be finite and positive");}

        this->declare_parameter("imu_msg_buffer_window_s", 10.0);
        imuMsgBufferWindowS_ = this->get_parameter("imu_msg_buffer_window_s").as_double();
        if (!std::isfinite(imuMsgBufferWindowS_) ||
            imuMsgBufferWindowS_ <= 0) {throw std::invalid_argument("imu_msg_buffer_window_s must be finite and positive");}
    }

    void VIONode::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        // Reject incorrect frame IDs
        if(!validateFrameID(msg->header.frame_id, imuFrameID_, "IMU")){return;}
        // Input validation
        const auto vector_is_finite = [](const geometry_msgs::msg::Vector3& v) {
            return std::isfinite(v.x) &&
                std::isfinite(v.y) &&
                std::isfinite(v.z);
        };
        // Not consuming currently
        // const auto covariance_is_finite = [](const std::array<double, 9>& covariance) {
        //     for (const double value : covariance) {
        //         if (!std::isfinite(value)) {
        //             return false;
        //         }
        //     }
        //     return true;
        // };

        bool isFinite = (
            vector_is_finite(msg->angular_velocity) &&
            vector_is_finite(msg->linear_acceleration));
            // Not consuming covariance currently
            // covariance_is_finite(msg->orientation_covariance) &&
            // covariance_is_finite(msg->angular_velocity_covariance) &&
            // covariance_is_finite(msg->linear_acceleration_covariance));
        std::lock_guard<std::mutex> lock(dataMutex_);
        if(isFinite)
        {
            rclcpp::Time msg_stamp(msg->header.stamp);
            if(imuBuffer_.empty()) {
                imuBuffer_.push_front(*msg);
            }
            else {
                rclcpp::Time buffer_latest_stamp(imuBuffer_.front().header.stamp);
                if(msg_stamp <= buffer_latest_stamp) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "IMU message rejected, current timestamp older than latest accepted. Current=%.9f, Latest=%.9f",
                        msg_stamp.seconds(), buffer_latest_stamp.seconds());
                    return;
                }
                const double imu_gap_s = (msg_stamp - buffer_latest_stamp).seconds();
                if(imu_gap_s > imuMsgGapThresholdS_) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "IMU message gap(%.3f) between samples greater than imu_msg_gap_threshold_s(%.3f)", imu_gap_s, imuMsgGapThresholdS_);
                }
                // Push then enforce our buffer window
                imuBuffer_.push_front(*msg);
                while(!imuBuffer_.empty() && (msg_stamp - rclcpp::Time(imuBuffer_.back().header.stamp)).seconds() >
                                                            imuMsgBufferWindowS_) {
                    imuBuffer_.pop_back();
                }
            }
        }
        else {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "IMU message rejected, message values nonfinite");
        }
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

    std::optional<ImuMeasurement> VIONode::interpolateImuMeasurement(
        const sensor_msgs::msg::Imu& before,
        const sensor_msgs::msg::Imu& after,
        const rclcpp::Time& target_stamp) const
    {
        // Validation
        // Ensure all stamps have same clock type
        rclcpp::Time beforeStamp(before.header.stamp);
        rclcpp::Time afterStamp(after.header.stamp);
        if(beforeStamp.get_clock_type() != afterStamp.get_clock_type() ||
           beforeStamp.get_clock_type() != target_stamp.get_clock_type()){return std::nullopt;}

        if(before.header.frame_id != imuFrameID_ || after.header.frame_id != imuFrameID_ ||
           afterStamp <= beforeStamp || target_stamp < beforeStamp ||
           target_stamp > afterStamp){return std::nullopt;}

        const auto vector_is_finite = [](const geometry_msgs::msg::Vector3& v) {
            return std::isfinite(v.x) &&
                std::isfinite(v.y) &&
                std::isfinite(v.z);
        };
        if(!vector_is_finite(before.angular_velocity) || !vector_is_finite(before.linear_acceleration) ||
           !vector_is_finite(after.angular_velocity) || !vector_is_finite(after.linear_acceleration)){return std::nullopt;}

        auto alpha = (target_stamp - beforeStamp).seconds() /
                     (afterStamp - beforeStamp).seconds();
        if(!std::isfinite(alpha) || alpha > 1.0 || alpha < 0.0){return std::nullopt;}
        auto interpolate = [&alpha](double before_value, double after_value) {
            return (1.0 - alpha) * before_value + alpha * after_value;
        };

        ImuMeasurement m;
        m.stamp = target_stamp;
        m.angular_velocity.x() = interpolate(before.angular_velocity.x, after.angular_velocity.x);
        m.angular_velocity.y() = interpolate(before.angular_velocity.y, after.angular_velocity.y);
        m.angular_velocity.z() = interpolate(before.angular_velocity.z, after.angular_velocity.z);
        m.linear_acceleration.x() = interpolate(before.linear_acceleration.x, after.linear_acceleration.x);
        m.linear_acceleration.y() = interpolate(before.linear_acceleration.y, after.linear_acceleration.y);
        m.linear_acceleration.z() = interpolate(before.linear_acceleration.z, after.linear_acceleration.z);
        if(!m.angular_velocity.allFinite() ||
            !m.linear_acceleration.allFinite()) {
            return std::nullopt;
        }
        return m;
    }

    std::optional<std::vector<ImuMeasurement>> VIONode::extractImuMeasurements(
        const std::deque<sensor_msgs::msg::Imu>& buffer,
        const rclcpp::Time& start_stamp,
        const rclcpp::Time& end_stamp) const
    {
        // The interval itself must be valid before any timestamp subtraction or
        // comparison is attempted. rclcpp throws if different clock types are
        // compared, so clock validation must happen first.
        if(start_stamp.get_clock_type() != end_stamp.get_clock_type() ||
           buffer.size() < std::size_t(2)){return std::nullopt;}
        if(end_stamp <= start_stamp){return std::nullopt;}
        // This buffer stores the latest sample at the front and the oldest at
        // the back. Both endpoints must be covered so interpolation never
        // becomes extrapolation.
        const rclcpp::Time oldest_stamp(buffer.back().header.stamp);
        const rclcpp::Time newest_stamp(buffer.front().header.stamp);
        if(oldest_stamp.get_clock_type() != start_stamp.get_clock_type() ||
           newest_stamp.get_clock_type() != start_stamp.get_clock_type()){
            return std::nullopt;
        }
        if(oldest_stamp > start_stamp || newest_stamp < end_stamp){
            return std::nullopt;
        }
        // Reverse iteration visits the newest-first deque chronologically. Each
        // adjacent pair describes one interval in which a boundary may fall.
        std::vector<ImuMeasurement> measurements;
        bool start_added = false;
        auto before = buffer.rbegin();
        auto after = std::next(before);
        for(; after != buffer.rend(); ++before, ++after) {
            const rclcpp::Time before_stamp(before->header.stamp);
            const rclcpp::Time after_stamp(after->header.stamp);
            // Enforce the deque's strictly increasing chronological invariant.
            // The interpolation helper validates frames and numeric values when
            // this pair contributes a measurement.
            if(before_stamp.get_clock_type() != start_stamp.get_clock_type() ||
               after_stamp.get_clock_type() != start_stamp.get_clock_type() ||
               after_stamp <= before_stamp){return std::nullopt;}
            // Add an interpolated measurement exactly at the beginning of the
            // requested interval. Equality handles a boundary that already
            // coincides with a raw IMU timestamp.
            if(!start_added &&
               before_stamp <= start_stamp &&
               start_stamp <= after_stamp) {
                const auto start_measurement =
                    interpolateImuMeasurement(*before, *after, start_stamp);
                if(!start_measurement){return std::nullopt;}
                measurements.push_back(*start_measurement);
                start_added = true;
            }
            if(!start_added){continue;}
            // Preserve each raw sample strictly inside the interval. Calling
            // the interpolation helper at after_stamp returns the exact raw
            // value while keeping all validation and conversion in one place.
            if(after_stamp > start_stamp && after_stamp < end_stamp) {
                const auto interior_measurement =
                    interpolateImuMeasurement(*before, *after, after_stamp);
                if(!interior_measurement){return std::nullopt;}
                measurements.push_back(*interior_measurement);
            }
            // Once this pair brackets the end, append the exact end boundary
            // and finish. Since end_stamp is strictly newer than start_stamp,
            // this cannot duplicate the first measurement.
            if(before_stamp <= end_stamp && end_stamp <= after_stamp) {
                const auto end_measurement =
                    interpolateImuMeasurement(*before, *after, end_stamp);
                if(!end_measurement){return std::nullopt;}
                measurements.push_back(*end_measurement);

                if(measurements.size() < std::size_t(2) ||
                   measurements.front().stamp != start_stamp ||
                   measurements.back().stamp != end_stamp){return std::nullopt;}
                return measurements;
            }
        }
        // Reaching the end means at least one requested boundary was not found,
        // despite the coarse oldest/newest bracketing check above.
        return std::nullopt;
    }

    std::optional<ImuWindowStatistics> VIONode::computeImuWindowStatistics(
        const std::vector<ImuMeasurement>& measurements) const
    {
        if(measurements.size() < std::size_t(2)){return std::nullopt;}

        const auto clock_type = measurements.front().stamp.get_clock_type();
        Eigen::Vector3d angular_velocity_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity_m2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_acceleration_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_acceleration_m2 = Eigen::Vector3d::Zero();

        // Welford's algorithm updates the mean and the sum of squared
        // deviations in one pass. M2/(N-1) is the per-axis sample variance.
        std::size_t sample_count = 0;
        for(std::size_t i = 0; i < measurements.size(); ++i) {
            const auto& measurement = measurements[i];
            if(measurement.stamp.get_clock_type() != clock_type ||
               !measurement.angular_velocity.allFinite() ||
               !measurement.linear_acceleration.allFinite()){return std::nullopt;}

            if(i > 0 && measurement.stamp <= measurements[i - 1].stamp){return std::nullopt;}

            ++sample_count;
            const double count = static_cast<double>(sample_count);

            const Eigen::Vector3d angular_delta =
                measurement.angular_velocity - angular_velocity_mean;
            angular_velocity_mean += angular_delta / count;
            const Eigen::Vector3d angular_delta_from_updated_mean =
                measurement.angular_velocity - angular_velocity_mean;
            angular_velocity_m2 += angular_delta.cwiseProduct(
                angular_delta_from_updated_mean);

            const Eigen::Vector3d acceleration_delta =
                measurement.linear_acceleration - linear_acceleration_mean;
            linear_acceleration_mean += acceleration_delta / count;
            const Eigen::Vector3d acceleration_delta_from_updated_mean =
                measurement.linear_acceleration - linear_acceleration_mean;
            linear_acceleration_m2 += acceleration_delta.cwiseProduct(
                acceleration_delta_from_updated_mean);
        }

        const double duration_s = (measurements.back().stamp - measurements.front().stamp).seconds();
        if(!std::isfinite(duration_s) || duration_s <= 0.0){return std::nullopt;}

        const double sample_variance_denominator = static_cast<double>(sample_count - 1);
        // M2 should be nonnegative. Clamp tiny negative roundoff before taking
        // the component-wise square root.
        const Eigen::Vector3d angular_velocity_variance =
            (angular_velocity_m2 / sample_variance_denominator).cwiseMax(0.0);
        const Eigen::Vector3d linear_acceleration_variance =
            (linear_acceleration_m2 / sample_variance_denominator).cwiseMax(0.0);

        ImuWindowStatistics statistics;
        statistics.sample_count = sample_count;
        statistics.duration_s = duration_s;
        statistics.angular_velocity_mean = angular_velocity_mean;
        statistics.angular_velocity_stddev =
            angular_velocity_variance.cwiseSqrt();
        statistics.linear_acceleration_mean = linear_acceleration_mean;
        statistics.linear_acceleration_stddev =
            linear_acceleration_variance.cwiseSqrt();

        if(!statistics.angular_velocity_mean.allFinite() ||
           !statistics.angular_velocity_stddev.allFinite() ||
           !statistics.linear_acceleration_mean.allFinite() ||
           !statistics.linear_acceleration_stddev.allFinite()){
            return std::nullopt;
        }

        return statistics;
    }

}   // namespace vio_node
