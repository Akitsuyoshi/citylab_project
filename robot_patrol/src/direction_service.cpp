#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_server") {
    std::string service_n = "/direction_service";
    service_ = create_service<std_srvs::srv::Trigger>(
        service_n,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          return service_callback(request, response);
        });
  }

private:
  void service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {}

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
