#include "custom_interfaces/srv/get_direction.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <geometry_msgs/msg/twist.hpp>
#include <iterator>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

struct Position {
  double x;
  double y;
};

enum MissionState {
  LEAVING,
  RETURNING,
  APPROACHING,
  COMPLETED,
  TURNING,
  TURNCOMPLETED
};

class Patrol : public rclcpp::Node {
  using GetDirection = custom_interfaces::srv::GetDirection;

public:
  Patrol() : Node("robot_patrol_with_service") {
    RCLCPP_INFO(get_logger(), "Here node is started");
    std::string name_service = "/direction_service";
    client_ = create_client<GetDirection>(name_service);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...",
                  name_service.c_str());
    }
    RCLCPP_INFO(get_logger(), "Service Client Ready");

    reentrant_group_1_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = reentrant_group_1_;
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          return laser_callback(msg);
        },
        sub_options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          return odom_callback(msg);
        },
        sub_options);
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { return move_around_robot(); }, reentrant_group_1_);
  };

  int get_lap_count() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return lap_count_;
  }

  void stop_robot() {
    RCLCPP_INFO(get_logger(), "Robot is stopped");
    geometry_msgs::msg::Twist cmd;
    pub_->publish(cmd);
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int size = static_cast<int>(msg->ranges.size()) - 1;
    // indx for forward +-30 degree on linear.x axis;
    int forward_start =
        std::clamp(static_cast<int>(std::round((-M_PI / 6 - msg->angle_min) /
                                               msg->angle_increment)),
                   0, size);
    int forward_end =
        std::clamp(static_cast<int>(std::round((M_PI / 6 - msg->angle_min) /
                                               msg->angle_increment)),
                   0, size);

    auto min_foward_distance = std::min_element(
        msg->ranges.begin() + forward_start, msg->ranges.begin() + forward_end);
    if (*min_foward_distance >= 0.4) {
      set_direction("forward");
    } else {
      auto request = std::make_shared<GetDirection::Request>();
      request->laser_data = *msg;
      auto fut = client_->async_send_request(
          request, [this](rclcpp::Client<GetDirection>::SharedFuture result) {
            auto response = result.get();
            set_direction(response->direction);
          });
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Position current_position;
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);

    if (!init_home_) {
      home_position_ = current_position;
      init_home_ = true;
      RCLCPP_INFO(get_logger(), "Home is set at (%.3f, %.3f)", home_position_.x,
                  home_position_.y);
    }

    MissionState mission_state = get_mission_state();
    double distance_to_home =
        calculate_distance(current_position, home_position_);
    if (mission_state == LEAVING) {
      if (distance_to_home > 1) {
        set_mission_state(RETURNING);
        RCLCPP_INFO(get_logger(), "Returning");
      }
    } else if (mission_state == RETURNING) {
      if (distance_to_home < 0.5) {
        set_mission_state(APPROACHING);
        RCLCPP_INFO(get_logger(), "Approaching");
      }
    } else if (mission_state == APPROACHING) {
      double target_angle = std::atan2(home_position_.y - current_position.y,
                                       home_position_.x - current_position.x);
      double err = norm_angle(target_angle - current_yaw_);
      if (std::fabs(err) > 0.2) {
        set_target_yaw(err);
      }
      if (distance_to_home < goal_tolerance_) {
        set_mission_state(COMPLETED);
        lap_count_++;
        RCLCPP_INFO(get_logger(), "Completed");
      }
    } else if (mission_state == COMPLETED) {
      set_target_yaw(norm_angle(current_yaw_ + M_PI));
      set_mission_state(TURNING);
      RCLCPP_INFO(get_logger(), "Turning");
    } else if (mission_state == TURNING) {
      if (std::fabs(norm_angle(target_yaw_ - current_yaw_)) < 0.05) {
        set_mission_state(TURNCOMPLETED);
        RCLCPP_INFO(get_logger(), "TurnCompleted");
      }
    } else { // mission_state == TURNCOMPLETED
      // start a new lap
      init_home_ = false;
      set_mission_state(LEAVING);
      RCLCPP_INFO(get_logger(), "Leaving");
    }
  }

  double norm_angle(double a) const {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  }

  void move_around_robot() {
    MissionState mission_state = get_mission_state();
    switch (mission_state) {
    case COMPLETED:
    case TURNCOMPLETED:
      stop_robot();
      break;
    case TURNING:
      turn_robot();
      break;
    case APPROACHING:
      approach_robot();
      break;
    default:
      move_robot();
      break;
    }
  }

  void move_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    std::string dir = get_direction();
    if (dir.empty()) {
      RCLCPP_INFO(get_logger(),
                  "Waiting for /scan topic, direction is not set yet");
      return;
    } else if (dir == "right") {
      cmd.angular.z = -0.6;
    } else if (dir == "forward") {
      cmd.angular.z = 0.0;
    } else if (dir == "left") {
      cmd.angular.z = 0.6;
    } else {
      RCLCPP_ERROR(get_logger(), "Unregisterd directioin is set: %s",
                   dir.c_str());
      stop_robot();
      return;
    }
    pub_->publish(cmd);
  }

  void turn_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.4;
    pub_->publish(cmd);
  }

  void approach_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    cmd.angular.z = std::clamp(1.0 * get_target_yaw(), -0.3, 0.3);
    pub_->publish(cmd);
  }

  double calculate_distance(const Position &a, const Position &b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  }

  void set_direction(std::string direction) {
    std::lock_guard<std::mutex> lck(mutex_);
    direction_ = direction;
  }

  std::string get_direction() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return direction_;
  }

  double get_target_yaw() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return target_yaw_;
  }

  void set_target_yaw(double target_yaw) {
    std::lock_guard<std::mutex> lck(mutex_);
    target_yaw_ = target_yaw;
  }

  MissionState get_mission_state() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return mission_state_;
  }

  void set_mission_state(MissionState mission_state) {
    std::lock_guard<std::mutex> lck(mutex_);
    mission_state_ = mission_state;
  }

  mutable std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_1_;
  rclcpp::Client<GetDirection>::SharedPtr client_;

  std::string direction_{""};
  Position home_position_;
  bool init_home_{false};
  MissionState mission_state_{LEAVING};
  double goal_tolerance_{0.2};
  int lap_count_{0};
  double current_yaw_{0.0};
  double target_yaw_{0.0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3);
  executor.add_node(node);
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    executor.spin_some();
    if (node->get_lap_count() >= 2) {
      node->stop_robot();
      rclcpp::shutdown();
    }
  }
  return 0;
}