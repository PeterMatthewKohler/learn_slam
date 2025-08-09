#ifndef TRAJECTORY_NODE_IMPL_HPP
#define TRAJECTORY_NODE_IMPL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <mutex>

namespace trajectory_node {

    struct TrajectoryPoint {
        double x;
        double y;
        double z;
        double yaw;
        double timestamp;
    };

    class TrajectoryNode : public rclcpp::Node
    {
        public:
        TrajectoryNode(const rclcpp::NodeOptions& options);

        private:
        // Service clients
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr trajectoryServiceClient;
        
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
        
        // Service server to trigger trajectory sending
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sendTrajectoryService;
        
        // Timer for periodic trajectory sending
        rclcpp::TimerBase::SharedPtr trajectoryTimer;
        
        // Parameters
        std::string trajectoryFilePath;
        std::string targetServiceName;
        double sendFrequency;
        bool autoSend;
        
        // Internal data
        std::vector<TrajectoryPoint> trajectory;
        bool trajectoryLoaded;
        
        // Mutex for thread safety
        std::mutex mutex_;
        
        // Methods
        bool loadTrajectoryFromJson(const std::string& filePath);
        void trajectoryTimerCallback();
        void sendTrajectoryServiceCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void sendTrajectory();
        nav_msgs::msg::Path convertToPath(const std::vector<TrajectoryPoint>& trajectory);
        void publishPath(const nav_msgs::msg::Path& path);
        
        // JSON parsing helpers
        TrajectoryPoint parseTrajectoryPoint(const nlohmann::json& pointJson);
    };
}

#endif // TRAJECTORY_NODE_IMPL_HPP
