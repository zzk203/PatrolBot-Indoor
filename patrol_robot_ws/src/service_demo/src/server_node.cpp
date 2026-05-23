#include "rclcpp/rclcpp.hpp"
#include "service_demo/srv/add_two_ints.hpp"

class AddTwoIntsServer : public rclcpp::Node {
public:
  AddTwoIntsServer() : Node("add_two_ints_server") {
    service_ = this->create_service<service_demo::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "AddTwoInts service ready");
  }

private:
  void handle_request(
      const std::shared_ptr<service_demo::srv::AddTwoInts::Request> request,
      std::shared_ptr<service_demo::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Request: %d + %d = %d",
                request->a, request->b, response->sum);
  }

  rclcpp::Service<service_demo::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsServer>());
  rclcpp::shutdown();
  return 0;
}
