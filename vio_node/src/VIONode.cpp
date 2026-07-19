#include <vio_node/VIONode.hpp>
#include <vio_node/Validation.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc.hpp>

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
        if(publishEstimatorDebug_) {
            debugVisualOdomPub_ =
                this->create_publisher<nav_msgs::msg::Odometry>(
                    "debug/visual_odom",
                    10
                );
            debugImuPredictionOdomPub_ =
                this->create_publisher<nav_msgs::msg::Odometry>(
                    "debug/imu_prediction_odom",
                    10
                );
        }
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


    void VIONode::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        // Reject incorrect frame IDs
        if(!validateFrameID(msg->header.frame_id, imuFrameID_, "IMU")){return;}
        // Input validation
        // Not consuming currently
        // const auto covariance_is_finite = [](const std::array<double, 9>& covariance) {
        //     for (const double value : covariance) {
        //         if (!std::isfinite(value)) {
        //             return false;
        //         }
        //     }
        //     return true;
        // };

        const bool measurement_is_finite =
            validation::isFinite(msg->angular_velocity) &&
            validation::isFinite(msg->linear_acceleration);
            // Not consuming covariance currently
            // covariance_is_finite(msg->orientation_covariance) &&
            // covariance_is_finite(msg->angular_velocity_covariance) &&
            // covariance_is_finite(msg->linear_acceleration_covariance));
        bool measurement_accepted = false;
        if(measurement_is_finite)
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
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
            measurement_accepted = true;
        }
        else {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "IMU message rejected, message values nonfinite");
        }

        if(measurement_accepted) {
            processPendingVisualMeasurements();
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
        // Reject backward timestamps using every accepted stereo pair, but
        // compare the processing interval against only the last pair that
        // passed this decimation guard.
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

            if(lastProcessedVisualStamp_) {
                const double time_since_last_processed_s =
                    (visual_stamp - *lastProcessedVisualStamp_).seconds();
                if(time_since_last_processed_s <
                   stereoProcessingIntervalS_) {
                    return;
                }
            }
            lastProcessedVisualStamp_.emplace(visual_stamp);
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
