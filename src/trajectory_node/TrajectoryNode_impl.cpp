#include "trajectory_node/TrajectoryNode_impl.hpp"
#include <fstream>
#include <iostream>

namespace trajectory_node
{
    TrajectoryNode::TrajectoryNode(const rclcpp::NodeOptions& options) 
        : Node("trajectory_node", options), trajectoryLoaded(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("trajectory_file_path", "");
        this->declare_parameter<std::string>("target_service_name", "/get_plan");
        this->declare_parameter<double>("send_frequency", 1.0);
        this->declare_parameter<bool>("auto_send", false);
        
        // Get parameters
        trajectoryFilePath = this->get_parameter("trajectory_file_path").as_string();
        targetServiceName = this->get_parameter("target_service_name").as_string();
        sendFrequency = this->get_parameter("send_frequency").as_double();
        autoSend = this->get_parameter("auto_send").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "TrajectoryNode initialized");
        RCLCPP_INFO(this->get_logger(), "Trajectory file: %s", trajectoryFilePath.c_str());
        RCLCPP_INFO(this->get_logger(), "Target service: %s", targetServiceName.c_str());
        
        // Create service client
        trajectoryServiceClient = this->create_client<nav_msgs::srv::GetPlan>(targetServiceName);
        
        // Create publisher
        pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/trajectory/path", 10);
        
        // Create service server to manually trigger trajectory sending
        sendTrajectoryService = this->create_service<std_srvs::srv::Trigger>(
            "send_trajectory", 
            std::bind(&TrajectoryNode::sendTrajectoryServiceCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Load trajectory from file if specified
        if (!trajectoryFilePath.empty()) {
            if (loadTrajectoryFromJson(trajectoryFilePath)) {
                RCLCPP_INFO(this->get_logger(), "Successfully loaded trajectory with %zu points", trajectory.size());
                trajectoryLoaded = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from file: %s", trajectoryFilePath.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No trajectory file specified. Use 'trajectory_file_path' parameter.");
        }
        
        // Create timer for automatic sending if enabled
        if (autoSend && trajectoryLoaded) {
            auto timer_period = std::chrono::duration<double>(1.0 / sendFrequency);
            trajectoryTimer = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                std::bind(&TrajectoryNode::trajectoryTimerCallback, this));
            RCLCPP_INFO(this->get_logger(), "Auto-send enabled with frequency: %.2f Hz", sendFrequency);
        }
    }

    bool TrajectoryNode::loadTrajectoryFromJson(const std::string& filePath)
    {
        try {
            std::ifstream file(filePath);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filePath.c_str());
                return false;
            }
            
            nlohmann::json jsonData;
            file >> jsonData;
            
            if (!jsonData.contains("trajectory")) {
                RCLCPP_ERROR(this->get_logger(), "JSON file does not contain 'trajectory' field");
                return false;
            }
            
            trajectory.clear();
            for (const auto& pointJson : jsonData["trajectory"]) {
                TrajectoryPoint point = parseTrajectoryPoint(pointJson);
                trajectory.push_back(point);
            }
            
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON file: %s", e.what());
            return false;
        }
    }

    TrajectoryPoint TrajectoryNode::parseTrajectoryPoint(const nlohmann::json& pointJson)
    {
        TrajectoryPoint point;
        
        // Required fields
        point.x = pointJson.at("x").get<double>();
        point.y = pointJson.at("y").get<double>();
        
        // Optional fields with defaults
        point.z = pointJson.value("z", 0.0);
        point.yaw = pointJson.value("yaw", 0.0);
        point.timestamp = pointJson.value("timestamp", 0.0);
        
        return point;
    }

    void TrajectoryNode::trajectoryTimerCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (trajectoryLoaded) {
            sendTrajectory();
        }
    }

    void TrajectoryNode::sendTrajectoryServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Suppress unused parameter warning
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!trajectoryLoaded) {
            response->success = false;
            response->message = "No trajectory loaded";
            RCLCPP_WARN(this->get_logger(), "Attempted to send trajectory but none is loaded");
            return;
        }
        
        sendTrajectory();
        response->success = true;
        response->message = "Trajectory sent successfully";
        RCLCPP_INFO(this->get_logger(), "Trajectory sent via service request");
    }

    void TrajectoryNode::sendTrajectory()
    {
        if (!trajectoryServiceClient->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service %s not available", targetServiceName.c_str());
            return;
        }
        
        // Create service request
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        
        // Set start pose (first point in trajectory)
        if (!trajectory.empty()) {
            request->start.header.stamp = this->get_clock()->now();
            request->start.header.frame_id = "map";
            request->start.pose.position.x = trajectory[0].x;
            request->start.pose.position.y = trajectory[0].y;
            request->start.pose.position.z = trajectory[0].z;
            
            // Convert yaw to quaternion
            request->start.pose.orientation.z = sin(trajectory[0].yaw / 2.0);
            request->start.pose.orientation.w = cos(trajectory[0].yaw / 2.0);
        }
        
        // Set goal pose (last point in trajectory)
        if (trajectory.size() > 1) {
            request->goal.header.stamp = this->get_clock()->now();
            request->goal.header.frame_id = "map";
            auto& lastPoint = trajectory.back();
            request->goal.pose.position.x = lastPoint.x;
            request->goal.pose.position.y = lastPoint.y;
            request->goal.pose.position.z = lastPoint.z;
            
            // Convert yaw to quaternion
            request->goal.pose.orientation.z = sin(lastPoint.yaw / 2.0);
            request->goal.pose.orientation.w = cos(lastPoint.yaw / 2.0);
        }
        
        // Set tolerance (can be made configurable)
        request->tolerance = 0.1;
        
        // Send async request
        auto future = trajectoryServiceClient->async_send_request(request);
        
        // Also publish the path for visualization
        auto path = convertToPath(trajectory);
        publishPath(path);
        
        RCLCPP_INFO(this->get_logger(), "Sent trajectory with %zu points to service %s", 
                   trajectory.size(), targetServiceName.c_str());
    }

    nav_msgs::msg::Path TrajectoryNode::convertToPath(const std::vector<TrajectoryPoint>& trajectory)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";
        
        for (const auto& point : trajectory) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = point.z;
            
            // Convert yaw to quaternion
            pose.pose.orientation.z = sin(point.yaw / 2.0);
            pose.pose.orientation.w = cos(point.yaw / 2.0);
            
            path.poses.push_back(pose);
        }
        
        return path;
    }

    void TrajectoryNode::publishPath(const nav_msgs::msg::Path& path)
    {
        pathPublisher->publish(path);
        RCLCPP_DEBUG(this->get_logger(), "Published path with %zu poses", path.poses.size());
    }
}
