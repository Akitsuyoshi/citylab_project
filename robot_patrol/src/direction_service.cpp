#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <utility>
#include <vector>

class DirectionService : public rclcpp::Node {
  using GetDirection = custom_interfaces::srv::GetDirection;

public:
  DirectionService() : Node("direction_service_server") {
    std::string service_n = "/direction_service";
    service_ = create_service<GetDirection>(
        service_n, [this](const std::shared_ptr<GetDirection::Request> request,
                          std::shared_ptr<GetDirection::Response> response) {
          return service_callback(request, response);
        });
    RCLCPP_INFO(get_logger(), "Service Server Ready");
  }

private:
  void service_callback(const std::shared_ptr<GetDirection::Request> request,
                        std::shared_ptr<GetDirection::Response> response) {
    RCLCPP_INFO(get_logger(), "Service Requested");

    const sensor_msgs::msg::LaserScan &msg = request->laser_data;
    if (!msg.angle_increment) {
      RCLCPP_WARN(get_logger(),
                  "angle_increment in laser_scan needs to be more than 0");
      return;
    }
    int start_idx = (-M_PI / 2 - msg.angle_min) / msg.angle_increment;
    int front_start = (-M_PI / 6 - msg.angle_min) / msg.angle_increment;
    int front_end = (M_PI / 6 - msg.angle_min) / msg.angle_increment;
    int end_idx = (M_PI / 2 - msg.angle_min) / msg.angle_increment;
    if (start_idx >= front_start || front_start >= front_end ||
        front_end >= end_idx) {
      RCLCPP_WARN(get_logger(), "angle_ranges cannot be split into 3 sections");
      return;
    }

    std::vector<std::pair<std::string, float>> dirs = {
        {"right", get_total_dist_sec(start_idx, front_start, msg.ranges)},
        {"front", get_total_dist_sec(front_start, front_end, msg.ranges)},
        {"left", get_total_dist_sec(front_end, end_idx, msg.ranges)},
    };

    auto max_it = std::max_element(
        dirs.begin(), dirs.end(),
        [](const auto &a, const auto &b) { return a.second < b.second; });

    response->direction = max_it->first;
    RCLCPP_INFO(get_logger(), "Service Completed");
  }

  float get_total_dist_sec(int start_idx, int end_idx,
                           const std::vector<float> &ranges) const {
    float sum = 0.0;
    for (int i = start_idx; i <= end_idx; i++) {
      float ray = ranges[i];
      if (std::isfinite(ray)) {
        sum += ray;
      }
    }
    return sum;
  }

  rclcpp::Service<GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
