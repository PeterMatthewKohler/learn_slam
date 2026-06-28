#ifndef VIO_NODE_IMPL_H
#define VIO_NODE_IMPL_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
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
#include <optional>



namespace vio_node {
    class VIONode : public rclcpp::Node
    {
        public:
        VIONode(const rclcpp::NodeOptions& options);

        private:
        void initPubSubs(); // Helper function
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vioOdomPub_;
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
        // Odometry (for initial snapshotting)
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

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
        void initializeRectification(const sensor_msgs::msg::CameraInfo& info,
                                     RectificationData& rect);

        cv::Mat cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat distortionFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat rectificationMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat projectionCameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        bool saveRectifiedImage(const cv::Mat& rectified_img,
                                const std::string& output_dir,
                                const std::string& filename);
        // TF2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        //rclcpp::TimerBase::SharedPtr timer_;
        // Internal states
        std::optional<sensor_msgs::msg::Imu> currentImu_;
        std::optional<sensor_msgs::msg::CameraInfo> currentLeftCamInfo_;
        std::optional<sensor_msgs::msg::CameraInfo> currentRightCamInfo_;
        RectificationData leftRectMap_;
        RectificationData rightRectMap_;
        std::optional<nav_msgs::msg::Odometry> initVehOdom_;
        std::optional<nav_msgs::msg::Odometry> currentVIOOdom_;
        // Mutex for thread safety
        std::mutex dataMutex_;
        // Parameters
        void initParameters();  // Helper function
        std::string imuSubTopicName_;
        std::string leftImgSubTopicName_;
        std::string leftCameraInfoSubTopicName_;
        std::string rightImgSubTopicName_;
        std::string rightCameraInfoSubTopicName_;
        std::string odomSubTopicName_;
        std::string vioPubTopicName_;

        bool saved_ = false;

    };

}   // namespace vio_node
# endif // VIO_NODE_IMPL_H