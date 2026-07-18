#include <vio_node/VIONode.hpp>
#include <vio_node/Validation.hpp>

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <map>
#include <queue>

namespace vio_node {
    // OpenCV Helper Functions
    std::vector<VisualCorrespondence> VIONode::buildVisualCorrespondences(const cv::Size& image_size) const
    {
        /* Iterate over tracked features and retain only tracks that satisfy
            - age >= 2
            - Current pixel coordinates are finite
            - Current pixel is inside image
            - Previous and current 3D coordinates are finite
            - Previous and current z are positive
            - depth_m is finite and positive
        */
       std::vector<VisualCorrespondence> visCorrs;
       for(const auto& feature : tracked_features_) {
            if(feature.age < 2){continue;}  // age >= 2
            if (!std::isfinite(feature.px_left_curr.x) ||
                !std::isfinite(feature.px_left_curr.y)) {continue;} // Current pixel coords are finite
            if(feature.px_left_curr.x >= image_size.width || feature.px_left_curr.x < 0 ||
               feature.px_left_curr.y >= image_size.height || feature.px_left_curr.y < 0){continue;} // Current pixel is inside image
            if(!(std::isfinite(feature.point_left_cam_prev.x) &&
                 std::isfinite(feature.point_left_cam_prev.y) &&
                 std::isfinite(feature.point_left_cam_prev.z) &&
                 std::isfinite(feature.point_left_cam_curr.x) &&
                 std::isfinite(feature.point_left_cam_curr.y) &&
                 std::isfinite(feature.point_left_cam_curr.z))){continue;} // Prev and curr 3d coords are finite
            if(!(feature.point_left_cam_prev.z > 0 &&
                 feature.point_left_cam_curr.z > 0)){continue;} // Prev and curr z are positive
            if(!std::isfinite(feature.depth_m) || feature.depth_m <= 0){continue;} // depth_m is finite and positive
            // Build our visual correspondence
            VisualCorrespondence v;
            v.id = feature.id;
            v.age = feature.age;
            v.point_prev = feature.point_left_cam_prev;
            v.pixel_curr = feature.px_left_curr;
            v.point_curr = feature.point_left_cam_curr;
            visCorrs.push_back(v);
       }
       return visCorrs;
    }

    std::vector<VisualCorrespondence> VIONode::selectSpatiallyBalancedCorrespondences(
        const std::vector<VisualCorrespondence>& correspondences,
        const cv::Size& image_size,
        int grid_rows,
        int grid_cols,
        std::size_t max_correspondences) const
    {
        // Return empty for invalid image dimensions, grid dimensions, or a zero limit
        if(!std::isfinite(image_size.width) || !std::isfinite(image_size.height) ||
           image_size.width <= 0 || image_size.height <= 0 ||
           grid_rows <= 0 || grid_cols <= 0 || max_correspondences <= 0 ||
           grid_rows > image_size.height || grid_cols > image_size.width){return {};}

        // Divide each image into grid cells
        // < Row, < Col, List of VisualCorrespondences @ (Row,Col) > >
        // Max heap sorting by age w/ id as deterministic tie breaker(lower id first)
        auto cmp = [](const VisualCorrespondence& a, const VisualCorrespondence& b) {
            if(a.age != b.age){return a.age < b.age;}   // Higher age first
            return a.id > b.id; // Lower ID first
        };
        std::map<std::size_t, std::map<std::size_t,
            std::priority_queue<VisualCorrespondence, std::vector<VisualCorrespondence>, decltype(cmp)>>> grid;
        for(const auto& c : correspondences) {
            // Get the grid location of the correspondence
            float px = c.pixel_curr.x, py = c.pixel_curr.y;
            if (!std::isfinite(px) || !std::isfinite(py) ||
                px < 0.0F || py < 0.0F ||
                px >= static_cast<float>(image_size.width) ||
                py >= static_cast<float>(image_size.height)){continue;}
            // Normalized indexing and clamp to final cell
            const std::size_t row = std::min(grid_rows - 1,
                static_cast<int>(py * grid_rows / image_size.height));
            const std::size_t col = std::min(grid_cols - 1,
                static_cast<int>(px * grid_cols / image_size.width));
            // Add to grid
            grid[row].try_emplace(col, cmp).first->second.push(c);
        }
        // Select correspondences round-robin from each non-empty cell
        std::vector<VisualCorrespondence> output;
        std::size_t count = 0;
        while (count < max_correspondences)
        {
            bool empty = true;
            for (auto& [row_index, row_map] : grid){
                for (auto& [col_index, queue] : row_map){
                    if (!queue.empty()){
                        output.push_back(queue.top());
                        queue.pop();
                        ++count;
                        empty = false;
                        if (count >= max_correspondences){break;}
                    }
                }
                if (count >= max_correspondences){break;}
            }
            if (empty){break;}
        }
        return output;
    }

    std::optional<VisualPoseEstimate> VIONode::estimateRelativeVisualPose(
        const std::vector<VisualCorrespondence>& correspondences,
        const StereoCalibration& calib) const
    {
        // Validate calibration w/ positive fx and fy
        if(!calib.initialized ||
           !std::isfinite(calib.cx) || !std::isfinite(calib.cy) ||
           !std::isfinite(calib.fx) || !std::isfinite(calib.fy) ||
           calib.fx <= 0.0 || calib.fy <= 0.0){return std::nullopt;}
        // Require atleast 6 correspondences
        const std::size_t min_size_threshold = 6;
        if(correspondences.size() < min_size_threshold){return std::nullopt;}
        // Build object points and image points
        std::vector<cv::Point3d> obj_points;
        std::vector<cv::Point2f> img_points;
        for(const auto& c : correspondences){
            obj_points.push_back(c.point_prev);
            img_points.push_back(c.pixel_curr);
        }
        // Construct rectified camera matrix from fx, fy, cx, cy
        cv::Mat K_rect = (cv::Mat_<double>(3, 3) <<
            calib.fx,   0.0,        calib.cx,
            0.0,        calib.fy,   calib.cy,
            0.0,        0.0,        1.0
        );
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        cv::Matx33d rotMat;
        std::vector<int> inliers;
        std::vector<cv::Point3d> inlier_object_points;
        std::vector<cv::Point2d> inlier_image_points;
        std::vector<cv::Point2d> projected_points;

        try {
            bool success = cv::solvePnPRansac(
                                obj_points,             // Object points
                                img_points,             // Image points
                                K_rect,                 // Camera Matrix
                                cv::Mat(),              // Distortion coeffs, empty since image is rectified
                                rvec,                   // rvec
                                tvec,                   // tvec
                                false,                  // useExtrinsicGuess
                                100,                    // iterationCount
                                2.0f,                   // reprojectionError
                                0.99,                   // confidence
                                inliers,                // inliers
                                cv::SOLVEPNP_ITERATIVE);// flags
            if(!success || inliers.size() < min_size_threshold){return std::nullopt;}
            if(!validation::isFinite(rvec) ||
               !validation::isFinite(tvec)) {return std::nullopt;}
            // Convert rvec to rotation matrix
            cv::Rodrigues(rvec, rotMat);
            // Check for non-finite values
            for (double value : rotMat.val) {
                if (!std::isfinite(value)) {
                    return std::nullopt;
                }
            }
            // Reproject each inlier point
            inlier_object_points.reserve(inliers.size());
            inlier_image_points.reserve(inliers.size());
            for(int index : inliers) {
                if (index < 0 ||
                    static_cast<std::size_t>(index) >= correspondences.size()) {
                    return std::nullopt;
                }
                const auto& correspondence =
                    correspondences[static_cast<std::size_t>(index)];

                inlier_object_points.push_back(correspondence.point_prev);
                inlier_image_points.emplace_back(
                    correspondence.pixel_curr.x,
                    correspondence.pixel_curr.y);
            }
            // Reproject points for RMSE quality calculation
            cv::projectPoints(
                inlier_object_points,   // object points
                rvec,                   // rvec
                tvec,                   // tvec
                K_rect,                 // Camera matrix
                cv::Mat(),              // No distortion since pixels are rectified
                projected_points);      // image points
            if (projected_points.size() != inlier_image_points.size() ||
                projected_points.empty()) {return std::nullopt;}
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "estimateRelativeVisualPose openCV error: " << e.what());
            return std::nullopt;
        }
        VisualPoseEstimate v;
        v.rotation_curr_from_prev = rotMat;
        v.translation_curr_from_prev = tvec;
        v.inlier_indices = inliers;
        // --- Calculate quality metrics ---
        // Inlier ratio
        v.inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(correspondences.size());
        // Reprojection RMSE
        double squared_error_sum = 0.0;
        for (std::size_t i = 0; i < projected_points.size(); ++i) {
            const double dx = projected_points[i].x - inlier_image_points[i].x;
            const double dy = projected_points[i].y - inlier_image_points[i].y;
            squared_error_sum += dx * dx + dy * dy;
        }
        const double reprojection_rmse_px = std::sqrt(squared_error_sum / projected_points.size());
        if (!std::isfinite(reprojection_rmse_px)) {return std::nullopt;}
        v.reprojection_rmse_px = reprojection_rmse_px;
        // Median 3D error
        std::vector<double> point_errors_m;
        point_errors_m.reserve(inliers.size());

        for (int index : inliers) {
            if (index < 0 ||
                static_cast<std::size_t>(index) >= correspondences.size()) {return std::nullopt;}

            const auto& correspondence = correspondences[static_cast<std::size_t>(index)];

            const cv::Vec3d point_prev(correspondence.point_prev.x,
                                       correspondence.point_prev.y,
                                       correspondence.point_prev.z);
            const cv::Vec3d point_curr(correspondence.point_curr.x,
                                       correspondence.point_curr.y,
                                       correspondence.point_curr.z);

            // Predict where the previous 3D point should be in Ck.
            const cv::Vec3d point_curr_predicted = rotMat * point_prev + tvec;
            // Calculate error between prediction and current
            const double error_m = cv::norm(point_curr_predicted - point_curr);
            // Validate
            if (!std::isfinite(error_m)) {return std::nullopt;}

            point_errors_m.push_back(error_m);
        }

        if (point_errors_m.empty()) {return std::nullopt;}
        // Sort
        std::sort(point_errors_m.begin(), point_errors_m.end());
        // Get median value from sorted array
        const std::size_t middle = point_errors_m.size() / 2;
        double median_3d_error_m = 0.0;
        if (point_errors_m.size() % 2 == 0) {
            median_3d_error_m =
                0.5 * (point_errors_m[middle - 1] +
                        point_errors_m[middle]);
        }
        else {
            median_3d_error_m = point_errors_m[middle];
        }
        // Validate
        if (!std::isfinite(median_3d_error_m)) {return std::nullopt;}
        // Store
        v.median_3d_error_m = median_3d_error_m;

        return v;
    }

    bool VIONode::passesVisualPoseQualityChecks(const VisualPoseEstimate& estimate) const
    {
        constexpr std::size_t min_inliers = 30;
        constexpr double min_inlier_ratio = 0.50;
        constexpr double max_reprojection_rmse_px = 1.0;
        constexpr double max_median_3d_error_m = 0.5;

        if(estimate.inlier_indices.size() < min_inliers){return false;}
        if(!std::isfinite(estimate.inlier_ratio) ||
           !std::isfinite(estimate.reprojection_rmse_px) ||
           !std::isfinite(estimate.median_3d_error_m)){return false;}
        if(estimate.inlier_ratio < 0.0 || estimate.inlier_ratio > 1.0 ||
           estimate.reprojection_rmse_px < 0.0 ||
           estimate.median_3d_error_m < 0.0){return false;}
        if(estimate.inlier_ratio < min_inlier_ratio ||
           estimate.reprojection_rmse_px > max_reprojection_rmse_px ||
           estimate.median_3d_error_m > max_median_3d_error_m){return false;}

        return true;
    }

    std::optional<VisualCameraPose> VIONode::composeVisualCameraPose(
        const VisualCameraPose& previous_pose,
        const VisualPoseEstimate& relative_pose) const
    {
        // Validate inputs.
        if(!validation::isFinite(relative_pose.rotation_curr_from_prev) ||
           !validation::isFinite(relative_pose.translation_curr_from_prev) ||
           !validation::isFinite(previous_pose.rotation_world_from_camera) ||
           !validation::isFinite(previous_pose.translation_world_from_camera)) {return std::nullopt;}

        const cv::Matx33d R_Cprev_Ck = relative_pose.rotation_curr_from_prev.t();
        const cv::Vec3d t_Cprev_Ck = -R_Cprev_Ck * relative_pose.translation_curr_from_prev;

        // Validate the inverted transform before composition.
        if(!validation::isFinite(R_Cprev_Ck) ||
           !validation::isFinite(t_Cprev_Ck)) {return std::nullopt;}

        // Compose
        const cv::Matx33d R_W_Ck =
            previous_pose.rotation_world_from_camera * R_Cprev_Ck;
        const cv::Vec3d t_W_Ck =
            previous_pose.translation_world_from_camera +
            previous_pose.rotation_world_from_camera * t_Cprev_Ck;

        // Validate outputs.
        if(!validation::isFinite(R_W_Ck) ||
           !validation::isFinite(t_W_Ck)) {return std::nullopt;}

        return VisualCameraPose{R_W_Ck, t_W_Ck};
    }

}   // namespace vio_node
