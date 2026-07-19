#ifndef VIO_NODE_HPP
#define VIO_NODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
// OpenCV
#include <opencv2/core.hpp>
#include "vio_node/EstimatorTypes.hpp"
#include "vio_node/ImuTypes.hpp"
#include "vio_node/VisionTypes.hpp"
// C++ Includes
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace vio_node {
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
        // --- CV ---
        void initializeRectification(const sensor_msgs::msg::CameraInfo& info,
                                     RectificationData& rect);
        void initializeStereoCalibration(const sensor_msgs::msg::CameraInfo& left_info,
                                        const sensor_msgs::msg::CameraInfo& right_info,
                                        StereoCalibration& calib);

        cv::Mat cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat distortionFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat rectificationMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        cv::Mat projectionCameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info);
        std::vector<cv::Point2f> detectLeftFeatures(const cv::Mat& leftRectMap,
                                                    const cv::Mat& mask,
                                                    int max_corners);
        void processRectifiedStereo(const cv::Mat& leftRectImg, const cv::Mat& rightRectImg,
                                    const rclcpp::Time& stamp);
        void trackExistingFeaturesTemporal(const cv::Mat& prev_left, const cv::Mat& curr_left);
        void updateStereoDepthForTrackedFeatures(const cv::Mat& curr_left, const cv::Mat& curr_right,
                                                 const StereoCalibration& calib);
        std::vector<StereoMatch> computeStereoMatches(const cv::Mat& left,
                                                      const cv::Mat& right,
                                                      const std::vector<cv::Point2f>& left_points,
                                                      const StereoCalibration& calib) const;
        cv::Mat buildFeatureDetectionMask(const cv::Size& image_size,
                                          const std::vector<TrackedFeature>& existing_features) const;
        void addNewTrackedFeatures(const cv::Mat& left, const cv::Mat& right,
                                   const StereoCalibration& calib, int max_new_features);
        std::vector<VisualCorrespondence> buildVisualCorrespondences(const cv::Size& image_size) const;
        std::vector<VisualCorrespondence> selectSpatiallyBalancedCorrespondences(
            const std::vector<VisualCorrespondence>& correspondences,
            const cv::Size& image_size,
            int grid_rows,
            int grid_cols,
            std::size_t max_correspondences) const;
        std::optional<VisualPoseEstimate> estimateRelativeVisualPose(
            const std::vector<VisualCorrespondence>& correspondences,
            const StereoCalibration& calib) const;
        bool passesVisualPoseQualityChecks(const VisualPoseEstimate& estimate) const;
        std::optional<VisualCameraPose> composeVisualCameraPose(
            const VisualCameraPose& previous_pose,
            const VisualPoseEstimate& relative_pose) const;
        std::optional<geometry_msgs::msg::Quaternion> quaternionFromRotationMatrix(
            const cv::Matx33d& rotation) const;
        cv::Mat makeStereoDebugImage(const cv::Mat& leftRectImg, const cv::Mat& rightRectImg,
                                     const std::vector<StereoFeature>& features);
        // --- IMU ---
        std::optional<ImuMeasurement> interpolateImuMeasurement(
            const sensor_msgs::msg::Imu& before,
            const sensor_msgs::msg::Imu& after,
            const rclcpp::Time& target_stamp) const;
        std::optional<std::vector<ImuMeasurement>> extractImuMeasurements(
            const std::deque<sensor_msgs::msg::Imu>& buffer,
            const rclcpp::Time& start_stamp,
            const rclcpp::Time& end_stamp) const;
        std::optional<ImuWindowStatistics> computeImuWindowStatistics(
            const std::vector<ImuMeasurement>& measurements) const;
        bool isImuWindowStationary(
            const ImuWindowStatistics& statistics) const;
        std::optional<ImuInitialization> computeImuInitialization(
            const ImuWindowStatistics& statistics,
            const rclcpp::Time& initialization_stamp) const;
        // --- Estimator ---
        std::optional<EstimatorState> makeInitialEstimatorState(
            const ImuInitialization& initialization) const;
        std::optional<EstimatorState> propagateEstimatorState(
            const EstimatorState& state,
            const ImuMeasurement& start_measurement,
            const ImuMeasurement& end_measurement,
            const Eigen::Vector3d& gravity_world) const;
        std::optional<FilterState> propagateEstimatorStateThroughMeasurements(
            const FilterState& initial_filter_state,
            const std::vector<ImuMeasurement>& measurements,
            const Eigen::Vector3d& gravity_world) const;
        std::optional<VisualCameraPose> visualCameraPoseFromEstimatorState(
            const EstimatorState& state,
            const geometry_msgs::msg::TransformStamped& imu_from_camera) const;
        std::optional<VisualPoseMeasurement> visualPoseMeasurementFromCameraPose(
            const VisualCameraPose& world_from_camera,
            const rclcpp::Time& stamp,
            const geometry_msgs::msg::TransformStamped& imu_from_camera) const;
        bool validateVisualPoseMeasurement(
            const VisualPoseMeasurement& measurement) const;
        bool enqueueVisualPoseMeasurement(
            const VisualPoseMeasurement& measurement);
        void processPendingVisualMeasurements();
        void tryInitializeEstimator(const rclcpp::Time& visual_stamp);

        // --- TF2 ---
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        void initTF();
        void tryInitializeExtrinsics();
        bool validateFrameID(const std::string& actual, const std::string& expected,
                             const std::string& sensor_name);
        rclcpp::TimerBase::SharedPtr extrinsicsInitTimer_;
        std::optional<geometry_msgs::msg::TransformStamped> imuFromLeftCamera_; // T_I_CL
        std::optional<geometry_msgs::msg::TransformStamped> imuFromBody_; // T_I_B
        bool extrinsicsInitialized_ = false;
        double tfStereoBaselineM_ = 0.0;
        bool validateTransform(const geometry_msgs::msg::TransformStamped& transform,
                               const std::string& expected_target,
                               const std::string& expected_source);
        bool validateStereoTransform(const geometry_msgs::msg::TransformStamped& left_from_right);
        bool transformToOpenCV(const geometry_msgs::msg::TransformStamped& transform,
                               cv::Matx33d& rotation_target_from_source,
                               cv::Vec3d& translation_target_from_source) const;
        std::optional<geometry_msgs::msg::Pose> bodyPoseFromEstimatorState(
            const EstimatorState& state,
            const geometry_msgs::msg::TransformStamped& imu_from_body) const;
        // Internal states
        std::deque<sensor_msgs::msg::Imu> imuBuffer_;   // Front is latest, back is oldest
        std::optional<EstimatorContext> estimatorContext_;
        std::optional<VisualPoseChain> visualPoseChain_;
        std::optional<sensor_msgs::msg::CameraInfo> currentLeftCamInfo_;
        std::optional<sensor_msgs::msg::CameraInfo> currentRightCamInfo_;
        std::optional<rclcpp::Time> lastVisualStamp_;
        RectificationData leftRectMap_;
        RectificationData rightRectMap_;
        StereoCalibration stereoCalib_;
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
        double imuMsgGapThresholdS_;
        double imuMsgBufferWindowS_;
        double imuInitializationWindowS_;
        std::size_t imuMinSamples_;
        double imuStationaryMaxGyroMeanNormRadS_;
        double imuStationaryMaxGyroStddevRadS_;
        double imuStationaryAccelStddevMS2_;
        double imuStationaryGravMagMS2_;
        double imuStationaryGravTolMS2_;
        InitialCovarianceParameters initialCovarianceParameters_;
        ImuNoiseParameters imuNoiseParameters_;
        std::size_t visualMeasurementQueueMaxSize_;
        double visualPositionStddevM_;
        double visualOrientationStddevRad_;
        double visualInnovationGateChi2_;
    };

}   // namespace vio_node
#endif  // VIO_NODE_HPP
