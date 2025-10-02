#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

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
                        std::shared_ptr<GetDirection::Response> response) {}

  rclcpp::Service<GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
