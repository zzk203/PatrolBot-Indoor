#include "rclcpp/rclcpp.hpp"
#include "service_demo/srv/add_two_ints.hpp"

class AddTwoIntsClient : public rclcpp::Node {
public:
  AddTwoIntsClient() : Node("add_two_ints_client") {
    client_ = this->create_client<service_demo::srv::AddTwoInts>("add_two_ints");
  }

  void send_request(int a, int b) {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<service_demo::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Result: %d + %d = %ld",
                  a, b, future.get()->sum);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
  }

private:
  rclcpp::Client<service_demo::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClient>();
  node->send_request(3, 5);
  rclcpp::shutdown();
  return 0;
}
