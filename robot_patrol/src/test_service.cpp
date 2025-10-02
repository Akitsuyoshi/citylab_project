#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

class DirectionServiceClient : public rclcpp::Node {
  using GetDirection = custom_interfaces::srv::GetDirection;
  using LaserScan = sensor_msgs::msg::LaserScan;

public:
  DirectionServiceClient()
      : Node("test_direction_service_client"), service_ready_(false) {
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    laser_sub_ = create_subscription<LaserScan>(
        "/scan", qos,
        [this](const LaserScan::SharedPtr msg) { return laser_callback(msg); });
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
    service_ready_ = true;
    RCLCPP_INFO(get_logger(), "Service Client Ready");
  }

private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  bool service_ready_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!service_ready_) {
      return;
    }
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;
    RCLCPP_INFO(get_logger(), "Service Request");
    auto fut = client_->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture result) {
          auto response = result.get();
          RCLCPP_INFO(get_logger(), "Service Response: %s",
                      response->direction.c_str());
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionServiceClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
