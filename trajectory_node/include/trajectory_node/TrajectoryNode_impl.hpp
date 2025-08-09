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
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr trajectoryServiceClient;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sendTrajectoryService;
        rclcpp::TimerBase::SharedPtr trajectoryTimer;
        std::string trajectoryFilePath;
        std::string targetServiceName;
        double sendFrequency{};
        bool autoSend{};
        std::vector<TrajectoryPoint> trajectory;
        bool trajectoryLoaded{};
        std::mutex mutex_;
        bool loadTrajectoryFromJson(const std::string& filePath);
        void trajectoryTimerCallback();
        void sendTrajectoryServiceCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void sendTrajectory();
        nav_msgs::msg::Path convertToPath(const std::vector<TrajectoryPoint>& trajectory);
        void publishPath(const nav_msgs::msg::Path& path);
        TrajectoryPoint parseTrajectoryPoint(const nlohmann::json& pointJson);
    };
}

#endif // TRAJECTORY_NODE_IMPL_HPP


