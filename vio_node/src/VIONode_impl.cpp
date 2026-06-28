#include <vio_node/VIONode_impl.hpp>

namespace vio_node {
    VIONode::VIONode(const rclcpp::NodeOptions& options) : Node("vio_node", options)
    {
        // Initialize parameters
        initParameters();

        // Initialize publishers and subscribers
        initPubSubs();
    }

    void VIONode::initPubSubs()
    {
        // Initialize publishers and subscribers
        auto sensorQOS = rclcpp::SensorDataQoS();

        vioOdomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(vioPubTopicName_, 10);
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

        leftCameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            leftCameraInfoSubTopicName_, 10, std::bind(&VIONode::leftCameraInfoCallback, this, std::placeholders::_1));


        rightCameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            rightCameraInfoSubTopicName_, 10, std::bind(&VIONode::rightCameraInfoCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odomSubTopicName_, 10, std::bind(&VIONode::odomCallback, this, std::placeholders::_1));
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

        this->declare_parameter("odom_sub_topic_name", "/chassis/odom");
        odomSubTopicName_ = this->get_parameter("odom_sub_topic_name").as_string();

        this->declare_parameter("vio_pub_topic_name", "/vio/odom");
        vioPubTopicName_ = this->get_parameter("vio_pub_topic_name").as_string();
    }

    void VIONode::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        currentImu_.emplace(*msg);
    }

    void VIONode::leftCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {   // These should never change/deviate at runtime
        if(!currentLeftCamInfo_.has_value()){
            std::lock_guard<std::mutex> lock(dataMutex_);
            currentLeftCamInfo_.emplace(*msg);
            initializeRectification(currentLeftCamInfo_.value(), leftRectMap_);
        }
    }

    void VIONode::rightCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {   // These should never change/deviate at runtime
        if(!currentRightCamInfo_.has_value()){
            std::lock_guard<std::mutex> lock(dataMutex_);
            currentRightCamInfo_.emplace(*msg);
            initializeRectification(currentRightCamInfo_.value(), rightRectMap_);
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
        // Simple check to ensure synchronization
        const auto left_stamp = rclcpp::Time(left_msg->header.stamp);
        const auto right_stamp = rclcpp::Time(right_msg->header.stamp);
        const double stereo_dt = std::abs((left_stamp - right_stamp).seconds());
        if (stereo_dt > 0.005) {
            RCLCPP_WARN(get_logger(), "Stereo pair too far apart: %.6f", stereo_dt);
            return;
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

        //processRectifiedStereo(left_rectified, right_rectified, left_msg->header.stamp);
    }

    void VIONode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {   // Single snapshot at boot up
        if(!initVehOdom_.has_value()){
            std::lock_guard<std::mutex> lock(dataMutex_);
            initVehOdom_.emplace(*msg);
        }
    }

}   // namespace vio_node