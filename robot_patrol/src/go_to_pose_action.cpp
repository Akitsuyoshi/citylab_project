#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0; // in radiant

  struct Delta {
    double distance;
    double angle_err; // in radiant
  };

  static double norm_angle(double a) {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  }

  Delta diff_to(const Pose2D &goal) const {
    double dx = goal.x - x;
    double dy = goal.y - y;
    double distance = std::hypot(dx, dy);

    double goal_angle = std::atan2(dy, dx);
    double angle_err = norm_angle(goal_angle - theta);

    return {distance, angle_err};
  }
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

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&GoToPose::move_to_desired_pos, this));
  }

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Action Called");
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
    using namespace std::placeholders;
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    std::lock_guard<std::mutex> lck(mutex_);
    goal_handle_ = goal_handle;
    // store the desired position
    desired_pos_.x = goal->goal_pos.x;
    desired_pos_.y = goal->goal_pos.y;
    // convert from degree to radiant
    desired_pos_.theta = goal->goal_pos.theta * M_PI / 180.0;

    // auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    goal_active_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    std::lock_guard<std::mutex> lck(mutex_);
    current_pos_.theta = yaw;
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
  }

  void move_to_desired_pos() {
    std::unique_lock<std::mutex> lck(mutex_);
    if (!goal_active_ || !goal_handle_) {
      return;
    }
    Pose2D::Delta delta = current_pos_.diff_to(desired_pos_);
    double yaw_err =
        Pose2D::norm_angle(desired_pos_.theta - current_pos_.theta);
    lck.unlock();

    geometry_msgs::msg::Twist cmd;
    if (delta.distance > goal_tolerance_) {
      cmd.linear.x = 0.2;
      cmd.angular.z = delta.angle_err * 0.8;
      pub_->publish(cmd);
    } else {
      if (std::abs(yaw_err) > 0.1) {
        cmd.linear.x = 0;
        cmd.angular.z = yaw_err * 0.8;
        pub_->publish(cmd);
        return;
      }
      pub_->publish(cmd);

      auto result = std::make_shared<GoToPoseAction::Result>();
      result->status = true;
      lck.lock();
      goal_handle_->succeed(result);
      goal_active_ = false;
      goal_handle_.reset();
      RCLCPP_INFO(get_logger(), "Action Completed");
    }
  }

  std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> goal_handle_;
  Pose2D desired_pos_;
  Pose2D current_pos_;
  double goal_tolerance_{0.1};
  bool goal_active_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}