#ifndef VIO_NODE_IMPL_H
#define VIO_NODE_IMPL_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// TF2
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// Eigen
#include <Eigen/Dense>
// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// C++ Includes
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

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

        struct TrackedFeature {
            int id = -1;

            cv::Point2f px_left_prev;
            cv::Point2f px_left_curr;
            cv::Point2f px_right_curr;

            double disparity = 0.0;
            double depth_m = 0.0;

            cv::Point3d point_left_cam_prev;
            cv::Point3d point_left_cam_curr;

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

        // OpenCV Helpers
        struct RectificationData {
            cv::Mat K;
            cv::Mat D;
            cv::Mat R;
            cv::Mat P_rect_3x3;

            cv::Mat map1;
            cv::Mat map2;

            bool initialized = false;
        };

    class VIONode : public rclcpp::Node
    {
        public:
        VIONode(const rclcpp::NodeOptions& options);

        private:
        void initPubSubs(); // Helper function
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vioOdomPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugStereoFeaturePub_;
        // Subscribers
        // IMU
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
        // Front Stereo Left Camera
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> leftImgSub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr leftCameraInfoSub_;
        void leftCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
        // Front Stereo Right Camera
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rightImgSub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rightCameraInfoSub_;
        void rightCameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
        // Camera message callback synchronization
        using StereoSyncPolicy = message_filters::sync_policies::ApproximateTime<
                                    sensor_msgs::msg::Image,
                                    sensor_msgs::msg::Image>;
        std::shared_ptr<message_filters::Synchronizer<StereoSyncPolicy>> sync_;
        void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);

        void initializeRectification(const sensor_msgs::msg::CameraInfo& info,
                                     RectificationData& rect);
        void initializeStereoCalibration(const sensor_msgs::msg::CameraInfo& left_info,
                                        const sensor_msgs::msg::CameraInfo& right_info,
                                        StereoCalibration& calib);

        cv::Mat cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat distortionFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat rectificationMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat projectionCameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        bool saveRectifiedImage(const cv::Mat& rectified_img,
                                const std::string& output_dir,
                                const std::string& filename);
        std::vector<cv::Point2f> detectLeftFeatures(const cv::Mat& leftRectMap,
                                                    const cv::Mat& mask,
                                                    int max_corners);
        void processRectifiedStereo(const cv::Mat& leftRectImg, const cv::Mat& rightRectImg,
                                    const rclcpp::Time& stamp);
        void trackExistingFeaturesTemporal(const cv::Mat& prev_left, const cv::Mat& curr_left);
        void updateStereoDepthForTrackedFeatures(const cv::Mat& curr_left, const cv::Mat& curr_right,
                                                 const StereoCalibration& calib);
        cv::Mat buildFeatureDetectionMask(const cv::Size& image_size,
                                          const std::vector<TrackedFeature>& existing_features) const;
        void addNewTrackedFeatures(const cv::Mat& left, const cv::Mat& right,
                                   const StereoCalibration& calib, int max_new_features);
        cv::Mat makeStereoDebugImage(const cv::Mat& leftRectImg, const cv::Mat& rightRectImg,
                                     const std::vector<StereoFeature>& features);

        // TF2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        void initTF();
        void tryInitializeExtrinsics();
        rclcpp::TimerBase::SharedPtr extrinsicsInitTimer_;
        std::optional<geometry_msgs::msg::TransformStamped> imuFromLeftCamera_; // T_I_CL
        std::optional<geometry_msgs::msg::TransformStamped> imuFromRightCamera_; // T_I_CR
        std::optional<geometry_msgs::msg::TransformStamped> imuFromBody_; // T_I_B
        bool extrinsicsInitialized_ = false;
        //rclcpp::TimerBase::SharedPtr timer_;
        // Internal states
        std::optional<sensor_msgs::msg::Imu> currentImu_;
        std::optional<sensor_msgs::msg::CameraInfo> currentLeftCamInfo_;
        std::optional<sensor_msgs::msg::CameraInfo> currentRightCamInfo_;
        RectificationData leftRectMap_;
        RectificationData rightRectMap_;
        StereoCalibration stereoCalib_;
        std::optional<nav_msgs::msg::Odometry> currentVIOOdom_;
        // Stereo tracking
        cv::Mat prev_left_rectified_;
        std::vector<TrackedFeature> tracked_features_;
        int next_feature_id_ = 0;
        int frame_idx_ = 0;
        // Mutex for thread safety
        std::mutex dataMutex_;
        // Parameters
        void initParameters();  // Helper function
        std::string imuSubTopicName_;
        std::string leftImgSubTopicName_;
        std::string leftCameraInfoSubTopicName_;
        std::string rightImgSubTopicName_;
        std::string rightCameraInfoSubTopicName_;
        std::string vioPubTopicName_;
        bool publishDebugStereoFeatures_;
        std::string worldFrameID_;
        std::string bodyFrameID_;
        std::string imuFrameID_;
        std::string leftCameraFrameID_;
        std::string rightCameraFrameID_;
        bool publishTF_;
        double cameraIMUTimeOffsetS_;

    };

}   // namespace vio_node
# endif // VIO_NODE_IMPL_H
