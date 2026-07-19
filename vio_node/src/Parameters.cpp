#include <vio_node/VIONode.hpp>
#include <vio_node/ErrorStateEkf.hpp>

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

        this->declare_parameter("publish_estimator_debug", false);
        publishEstimatorDebug_ = this->get_parameter("publish_estimator_debug").as_bool();

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

        this->declare_parameter("stereo_processing_interval_s", 0.0);
        stereoProcessingIntervalS_ =
            this->get_parameter("stereo_processing_interval_s").as_double();
        if(!std::isfinite(stereoProcessingIntervalS_) ||
           stereoProcessingIntervalS_ < 0.0) {
            throw std::invalid_argument(
                "stereo_processing_interval_s must be finite and nonnegative");
        }

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

        this->declare_parameter("initial_position_stddev_m", 0.01);
        initialCovarianceParameters_.position_stddev_m =
            this->get_parameter("initial_position_stddev_m").as_double();
        requireValidStandardDeviation(
            initialCovarianceParameters_.position_stddev_m,
            "initial_position_stddev_m"
        );

        this->declare_parameter("initial_orientation_stddev_rad", 0.01);
        initialCovarianceParameters_.orientation_stddev_rad =
            this->get_parameter("initial_orientation_stddev_rad").as_double();
        requireValidStandardDeviation(
            initialCovarianceParameters_.orientation_stddev_rad,
            "initial_orientation_stddev_rad"
        );

        this->declare_parameter("initial_velocity_stddev_m_s", 0.05);
        initialCovarianceParameters_.velocity_stddev_m_s =
            this->get_parameter("initial_velocity_stddev_m_s").as_double();
        requireValidStandardDeviation(
            initialCovarianceParameters_.velocity_stddev_m_s,
            "initial_velocity_stddev_m_s"
        );

        this->declare_parameter("initial_gyroscope_bias_stddev_rad_s", 0.001);
        initialCovarianceParameters_.gyroscope_bias_stddev_rad_s =
            this->get_parameter(
                "initial_gyroscope_bias_stddev_rad_s").as_double();
        requireValidStandardDeviation(
            initialCovarianceParameters_.gyroscope_bias_stddev_rad_s,
            "initial_gyroscope_bias_stddev_rad_s"
        );

        this->declare_parameter(
            "initial_accelerometer_bias_stddev_m_s2",
            0.1
        );
        initialCovarianceParameters_.accelerometer_bias_stddev_m_s2 =
            this->get_parameter(
                "initial_accelerometer_bias_stddev_m_s2").as_double();
        requireValidStandardDeviation(
            initialCovarianceParameters_.accelerometer_bias_stddev_m_s2,
            "initial_accelerometer_bias_stddev_m_s2"
        );

        this->declare_parameter(
            "imu_gyroscope_noise_density_rad_s_sqrt_hz",
            1.0e-4
        );
        imuNoiseParameters_.gyroscope_noise_density_rad_s_sqrt_hz =
            this->get_parameter(
                "imu_gyroscope_noise_density_rad_s_sqrt_hz").as_double();

        this->declare_parameter(
            "imu_accelerometer_noise_density_m_s2_sqrt_hz",
            1.0e-3
        );
        imuNoiseParameters_.accelerometer_noise_density_m_s2_sqrt_hz =
            this->get_parameter(
                "imu_accelerometer_noise_density_m_s2_sqrt_hz").as_double();

        this->declare_parameter(
            "imu_gyroscope_bias_random_walk_rad_s2_sqrt_hz",
            1.0e-6
        );
        imuNoiseParameters_.gyroscope_bias_random_walk_rad_s2_sqrt_hz =
            this->get_parameter(
                "imu_gyroscope_bias_random_walk_rad_s2_sqrt_hz").as_double();

        this->declare_parameter(
            "imu_accelerometer_bias_random_walk_m_s3_sqrt_hz",
            1.0e-5
        );
        imuNoiseParameters_.accelerometer_bias_random_walk_m_s3_sqrt_hz =
            this->get_parameter(
                "imu_accelerometer_bias_random_walk_m_s3_sqrt_hz").as_double();

        if(!error_state_ekf::validateImuNoiseParameters(
               imuNoiseParameters_)) {
            throw std::invalid_argument(
                "IMU noise parameters must be finite, nonnegative, and produce finite squared values"
            );
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

        this->declare_parameter("visual_innovation_gate_chi2", 50.0);
        visualInnovationGateChi2_ =
            this->get_parameter("visual_innovation_gate_chi2").as_double();
        if(!std::isfinite(visualInnovationGateChi2_) ||
           visualInnovationGateChi2_ <= 0.0) {
            throw std::invalid_argument(
                "visual_innovation_gate_chi2 must be finite and positive");
        }

        this->declare_parameter("visual_reacquisition_rejection_count", 5);
        const auto visual_reacquisition_rejection_count =
            this->get_parameter(
                "visual_reacquisition_rejection_count").as_int();
        if(visual_reacquisition_rejection_count < 1) {
            throw std::invalid_argument(
                "visual_reacquisition_rejection_count must be at least 1");
        }
        visualReacquisitionRejectionCount_ = static_cast<std::size_t>(
            visual_reacquisition_rejection_count
        );

        this->declare_parameter(
            "visual_reacquisition_covariance_scale",
            10.0
        );
        visualReacquisitionCovarianceScale_ = this->get_parameter(
            "visual_reacquisition_covariance_scale").as_double();
        if(!std::isfinite(visualReacquisitionCovarianceScale_) ||
           visualReacquisitionCovarianceScale_ <= 1.0) {
            throw std::invalid_argument(
                "visual_reacquisition_covariance_scale must be finite and greater than 1");
        }

        this->declare_parameter(
            "visual_reacquisition_max_position_correction_m",
            0.2
        );
        visualReacquisitionMaxPositionCorrectionM_ = this->get_parameter(
            "visual_reacquisition_max_position_correction_m").as_double();
        requireFinitePositive(
            visualReacquisitionMaxPositionCorrectionM_,
            "visual_reacquisition_max_position_correction_m"
        );

        this->declare_parameter(
            "visual_reacquisition_max_orientation_correction_rad",
            0.1
        );
        visualReacquisitionMaxOrientationCorrectionRad_ =
            this->get_parameter(
                "visual_reacquisition_max_orientation_correction_rad").as_double();
        requireFinitePositive(
            visualReacquisitionMaxOrientationCorrectionRad_,
            "visual_reacquisition_max_orientation_correction_rad"
        );

    }
}   // namespace vio_node
