#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
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
public:
  Patrol() : Node("patrol_node"), direction_(0.0f) {
    RCLCPP_INFO(get_logger(), "Here node is started");
    reentrant_group_1_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = reentrant_group_1_;
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    _laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          return this->laser_callback(msg);
        },
        sub_options);
    _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          return this->odom_callback(msg);
        },
        sub_options);
    _pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _timer = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { return this->move_around_robot(); }, reentrant_group_1_);
  };

  int get_lap_count() {
    std::lock_guard<std::mutex> lck(mutex_);
    return lap_count_;
  }

  void stop_robot() {
    RCLCPP_INFO(get_logger(), "Robot is stopped");
    geometry_msgs::msg::Twist cmd;
    _pub->publish(cmd);
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // angle_min + -90_deg_idx * angle_increment = -90 degree
    // angle_min + 90_deg_idx * angle_increment = 90 degree
    int start_idx = (-M_PI / 2 - msg->angle_min) / msg->angle_increment;
    start_idx = std::max(0, start_idx);
    int end_idx = (M_PI / 2 - msg->angle_min) / msg->angle_increment;
    end_idx = std::min((int)msg->ranges.size() - 1, end_idx);
    if (start_idx >= end_idx) {
      stop_robot();
      return;
    }

    // indx for forward +-20 degree on linear.x axis;
    int forward_start = (-M_PI / 9 - msg->angle_min) / msg->angle_increment;
    int forward_end = (M_PI / 9 - msg->angle_min) / msg->angle_increment;
    // check forward(+-18 degree) closest obstacle
    auto min_foward_distance = std::min_element(
        msg->ranges.begin() + forward_start, msg->ranges.begin() + forward_end);
    if (*min_foward_distance >= 0.35) {
      set_direction(0.0f);
    } else {
      //  check foward(+-90 degree) closest obstacle
      auto min_distance = std::min_element(msg->ranges.begin() + start_idx,
                                           msg->ranges.begin() + end_idx);
      if (*min_distance < 0.25) {
        int min_idx = std::distance(msg->ranges.begin(), min_distance);
        float min_angle = msg->angle_min + min_idx * msg->angle_increment;
        float dir = (min_angle < 0) ? +M_PI / 2 : -M_PI / 2;
        set_direction(dir);
      } else {
        int max_idx = get_opened_ray_idx(start_idx, end_idx, msg->ranges);
        if (max_idx != -1) {
          set_direction(msg->angle_min + max_idx * msg->angle_increment);
        } else {
          stop_robot();
        }
      }
    }
  }

  int get_opened_ray_idx(int start_idx, int end_idx,
                         const std::vector<float> &ranges) const {
    float max_distance = 0.0;
    int max_idx = -1;
    for (int i = start_idx; i <= end_idx; i++) {
      float ray = ranges[i];
      if (std::isfinite(ray) && ray > max_distance) {
        max_distance = ray;
        max_idx = i;
      }
    }
    return max_idx;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);

    if (!init_home_) {
      home_position_ = current_position_;
      init_home_ = true;
      RCLCPP_INFO(get_logger(), "Home is set at (%.3f, %.3f)", home_position_.x,
                  home_position_.y);
    }

    MissionState mission_state = get_mission_state();
    double distance_to_home =
        calculate_distance(current_position_, home_position_);
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
      double target_angle = std::atan2(home_position_.y - current_position_.y,
                                       home_position_.x - current_position_.x);
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

  double norm_angle(double a) {
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
    cmd.angular.z = get_direction() / 2;
    _pub->publish(cmd);
  }

  void turn_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.4;
    _pub->publish(cmd);
  }

  void approach_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    cmd.angular.z = std::clamp(1.0 * get_target_yaw(), -0.3, 0.3);
    _pub->publish(cmd);
  }

  double calculate_distance(const Position &a, const Position &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  }

  void set_direction(float angle) {
    // // normalize angle
    if (angle > M_PI / 2) {
      angle = M_PI / 2;
    }
    if (angle < -M_PI / 2) {
      angle = -M_PI / 2;
    }
    std::lock_guard<std::mutex> lck(mutex_);
    direction_ = angle;
  }

  float get_direction() {
    std::lock_guard<std::mutex> lck(mutex_);
    return direction_;
  }

  double get_target_yaw() {
    std::lock_guard<std::mutex> lck(mutex_);
    return target_yaw_;
  }

  void set_target_yaw(double target_yaw) {
    std::lock_guard<std::mutex> lck(mutex_);
    target_yaw_ = target_yaw;
  }

  MissionState get_mission_state() {
    std::lock_guard<std::mutex> lck(mutex_);
    return mission_state_;
  }

  void set_mission_state(MissionState mission_state) {
    std::lock_guard<std::mutex> lck(mutex_);
    mission_state_ = mission_state;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub;
  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::CallbackGroup::SharedPtr reentrant_group_1_;
  float direction_;
  std::mutex mutex_;
  Position home_position_;
  Position current_position_;
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