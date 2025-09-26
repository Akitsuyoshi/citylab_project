#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
  };

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_distance =
        *std::min_element(msg->ranges.begin(), msg->ranges.end());
    RCLCPP_INFO(this->get_logger(), "Min distance: %.2f meters", min_distance);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}