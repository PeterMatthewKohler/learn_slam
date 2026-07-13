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
        auto sensorQOS = rclcpp::SensorDataQoS();
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imuSubTopicName_, 10, std::bind(&VIONode::imuCallback, this, std::placeholders::_1));
        // Synchronize stereo camera image subscribers
        leftImgSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, leftImgSubTopicName_, sensorQOS.get_rmw_qos_profile()
        );
        rightImgSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, rightImgSubTopicName_, sensorQOS.get_rmw_qos_profile()
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

            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                imuFromLeftCamera_ = std::move(imu_from_left);
                imuFromRightCamera_ = std::move(imu_from_right);
                imuFromBody_ = std::move(imu_from_body);
                extrinsicsInitialized_ = true;
            }

            extrinsicsInitTimer_->cancel();
            RCLCPP_INFO(get_logger(), "Cached VIO sensor extrinsics");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting For VIO Sensor Extrinsics: %s", ex.what());
        }
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
    }

    void VIONode::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        currentImu_.emplace(*msg);
    }

    void VIONode::leftCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {   // These should never change/deviate at runtime
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
    {   // These should never change/deviate at runtime
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
        if (!leftRectMap_.initialized || !rightRectMap_.initialized) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Waiting for rectification maps..."
            );
            return;
        }
        // DEBUG SLOW RUNRATE
        static size_t callback_count = 0;
        static size_t rejected_dt_count = 0;
        static size_t processed_count = 0;
        callback_count++;

        // Simple check to ensure synchronization
        const auto left_stamp = rclcpp::Time(left_msg->header.stamp);
        const auto right_stamp = rclcpp::Time(right_msg->header.stamp);
        const double stereo_dt = std::abs((left_stamp - right_stamp).seconds());
        if (stereo_dt > 0.010) {
            rejected_dt_count++;
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Rejecting stereo pair: dt=%.6f, callbacks=%zu rejected_dt=%zu processed=%zu",
                stereo_dt,
                callback_count,
                rejected_dt_count,
                processed_count
            );
            return;
        }
        processed_count++;
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

        processRectifiedStereo(left_rectified, right_rectified, left_msg->header.stamp);
    }

}   // namespace vio_node
