#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    RCLCPP_INFO(get_logger(), "Here node is started");
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    _laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          return this->laser_callback(msg);
        });
    _pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  };

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // angle_min: -3.141590118408203 = -180 deg
    // angle_max: 3.141590118408203 = 180 deg
    // angle_increment: 0.06346646696329117 = 3.6 deg
    // angle_min + -90_deg_idx * angle_increment = -90 degree
    // angle_min + 90_deg_idx * angle_increment = 90 degree
    int start_idx = (-M_PI / 2 - msg->angle_min) / msg->angle_increment;
    int end_idx = (M_PI / 2 - msg->angle_min) / msg->angle_increment;

    auto start_it = msg->ranges.begin() + start_idx;
    auto end_it = msg->ranges.begin() + end_idx;

    float min_distance = *std::min_element(start_it, end_it);
    if (min_distance >= 0.35) {
      move_forward();
    } else {
      float max_distance = 0.0;
      int max_idx = -1;
      for (size_t i = start_idx; i <= end_idx; i++) {
      float current_max = msg->ranges[i];
        if (std::isfinite(r) && r > max_distance) {
            max_distance = current_max;
            max_idx = i;
        }
      }

      if (max_idx != -1) {
        direction_ = msg->angle_min + max_idx * msg->angle_increment;        
      }
    }
    RCLCPP_INFO(get_logger(),
                "Min distance: %.2f m in a indices, between %d - %d",
                min_distance, start_idx, end_idx);
  }

  void move_forward() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    _pub->publish(cmd);
  }

  void stop() {
    geometry_msgs::msg::Twist cmd;
    _pub->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub;
  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}