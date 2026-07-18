#include <vio_node/VIONode.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace {
    void requireFinitePositive(double value, const char* parameter_name)
    {
        if(!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                std::string(parameter_name) + " must be finite and positive");
        }
    }

    void requireValidStandardDeviation(
        double value,
        const char* parameter_name)
    {
        const double variance = value * value;
        if(!std::isfinite(value) || value <= 0.0 ||
           !std::isfinite(variance) || variance <= 0.0) {
            throw std::invalid_argument(
                std::string(parameter_name) +
                " must be finite, positive, and produce a finite positive variance");
        }
    }
}  // namespace

namespace vio_node {
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
        requireFinitePositive(imuMsgGapThresholdS_, "imu_msg_gap_threshold_s");

        this->declare_parameter("imu_msg_buffer_window_s", 10.0);
        imuMsgBufferWindowS_ = this->get_parameter("imu_msg_buffer_window_s").as_double();
        requireFinitePositive(imuMsgBufferWindowS_, "imu_msg_buffer_window_s");

        this->declare_parameter("imu_initialization_window_s", 2.0);
        imuInitializationWindowS_ = this->get_parameter("imu_initialization_window_s").as_double();
        if (!std::isfinite(imuInitializationWindowS_) ||
            imuInitializationWindowS_ <= 0 ||
            imuInitializationWindowS_ > imuMsgBufferWindowS_) {
            throw std::invalid_argument(
                "imu_initialization_window_s must be finite, positive, and no larger than imu_msg_buffer_window_s");
        }

        this->declare_parameter("imu_initialization_min_samples", 100);
        const auto imu_min_samples =
            this->get_parameter("imu_initialization_min_samples").as_int();
        if(imu_min_samples < 2) {
            throw std::invalid_argument(
                "imu_initialization_min_samples must be at least 2");
        }
        imuMinSamples_ = static_cast<std::size_t>(imu_min_samples);

        this->declare_parameter("imu_stationary_max_gyro_mean_norm_rad_s", 0.005);
        imuStationaryMaxGyroMeanNormRadS_ =
            this->get_parameter("imu_stationary_max_gyro_mean_norm_rad_s").as_double();
        requireFinitePositive(
            imuStationaryMaxGyroMeanNormRadS_,
            "imu_stationary_max_gyro_mean_norm_rad_s"
        );

        this->declare_parameter("imu_stationary_max_gyro_stddev_rad_s", 0.001);
        imuStationaryMaxGyroStddevRadS_ =
            this->get_parameter("imu_stationary_max_gyro_stddev_rad_s").as_double();
        requireFinitePositive(
            imuStationaryMaxGyroStddevRadS_,
            "imu_stationary_max_gyro_stddev_rad_s"
        );

        this->declare_parameter("imu_stationary_max_accel_stddev_m_s2", 0.01);
        imuStationaryAccelStddevMS2_ =
            this->get_parameter("imu_stationary_max_accel_stddev_m_s2").as_double();
        requireFinitePositive(
            imuStationaryAccelStddevMS2_,
            "imu_stationary_max_accel_stddev_m_s2"
        );

        this->declare_parameter("imu_stationary_gravity_magnitude_m_s2", 9.8);
        imuStationaryGravMagMS2_ =
            this->get_parameter("imu_stationary_gravity_magnitude_m_s2").as_double();
        requireFinitePositive(
            imuStationaryGravMagMS2_,
            "imu_stationary_gravity_magnitude_m_s2"
        );

        this->declare_parameter("imu_stationary_gravity_tolerance_m_s2", 0.1);
        imuStationaryGravTolMS2_ =
            this->get_parameter("imu_stationary_gravity_tolerance_m_s2").as_double();
        if(!std::isfinite(imuStationaryGravTolMS2_) ||
           imuStationaryGravTolMS2_ <= 0.0 ||
           imuStationaryGravTolMS2_ >= imuStationaryGravMagMS2_) {
            throw std::invalid_argument(
                "imu_stationary_gravity_tolerance_m_s2 must be finite, positive, and smaller than imu_stationary_gravity_magnitude_m_s2");
        }

        this->declare_parameter("visual_measurement_queue_max_size", 20);
        const auto visual_measurement_queue_max_size =
            this->get_parameter("visual_measurement_queue_max_size").as_int();
        if(visual_measurement_queue_max_size < 1) {
            throw std::invalid_argument(
                "visual_measurement_queue_max_size must be at least 1");
        }
        visualMeasurementQueueMaxSize_ =
            static_cast<std::size_t>(visual_measurement_queue_max_size);

        this->declare_parameter("visual_position_stddev_m", 0.05);
        visualPositionStddevM_ =
            this->get_parameter("visual_position_stddev_m").as_double();
        requireValidStandardDeviation(
            visualPositionStddevM_,
            "visual_position_stddev_m"
        );

        this->declare_parameter("visual_orientation_stddev_rad", 0.035);
        visualOrientationStddevRad_ =
            this->get_parameter("visual_orientation_stddev_rad").as_double();
        requireValidStandardDeviation(
            visualOrientationStddevRad_,
            "visual_orientation_stddev_rad"
        );
    }
}   // namespace vio_node
