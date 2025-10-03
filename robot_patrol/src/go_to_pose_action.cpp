#include "custom_interfaces/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>

class GoToPose : public rclcpp::Node {
public:
  GoToPose() : Node("action_server") {
    RCLCPP_INFO(get_logger(), "Action server is created");
  }

private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}