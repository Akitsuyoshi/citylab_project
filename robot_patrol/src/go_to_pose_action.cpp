#include "custom_interfaces/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

struct Pose2D {
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
};

class GoToPose : public rclcpp::Node {
  using GoToPoseAction = custom_interfaces::action::GoToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

public:
  GoToPose() : Node("action_server") {
    using namespace std::placeholders;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Action Server Ready");
  }

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(get_logger(), "Received cancel goal request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    // store the desired position
    desired_pos_.x = goal->goal_pos.x;
    desired_pos_.y = goal->goal_pos.y;
    desired_pos_.theta = goal->goal_pos.theta;

    // auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_pos_.theta = yaw * 180.0 / M_PI;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  Pose2D desired_pos_;
  Pose2D current_pos_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}