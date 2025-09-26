#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node"), direction_(0.0f) {
    RCLCPP_INFO(get_logger(), "Here node is started");
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    _laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          return this->laser_callback(msg);
        });
    _pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _timer = create_wall_timer(std::chrono::milliseconds(100),
                               [this]() { return this->move_robot(); });
  };
  ~Patrol() { stop_robot(); };

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

    // indx for forward +- 30 degree;
    int foward_start = (-M_PI / 6 - msg->angle_min) / msg->angle_increment;
    int foward_end = (M_PI / 6 - msg->angle_min) / msg->angle_increment;
    float min_distance = *std::min_element(msg->ranges.begin() + foward_start,
                                           msg->ranges.begin() + foward_end);
    if (min_distance >= 0.35) {
      set_direction(0.0f);
    } else {
      float max_distance = 0.0;
      int max_idx = -1;
      for (int i = start_idx; i <= end_idx; i++) {
        float ray = msg->ranges[i];
        if (std::isfinite(ray) && ray > max_distance) {
          max_distance = ray;
          max_idx = i;
        }
      }

      if (max_idx != -1) {
        set_direction(msg->angle_min + max_idx * msg->angle_increment);
      } else {
        stop_robot();
      }
    }
  }

  void move_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    cmd.angular.z = get_direction() / 2;
    _pub->publish(cmd);
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
    RCLCPP_INFO(get_logger(), "Angle is set at : %.2f", angle);
  }

  float get_direction() {
    std::lock_guard<std::mutex> lck(mutex_);
    return direction_;
  }

  void stop_robot() {
    RCLCPP_INFO(get_logger(), "Robot is stopped");
    geometry_msgs::msg::Twist cmd;
    _pub->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub;
  rclcpp::TimerBase::SharedPtr _timer;
  float direction_;
  std::mutex mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}