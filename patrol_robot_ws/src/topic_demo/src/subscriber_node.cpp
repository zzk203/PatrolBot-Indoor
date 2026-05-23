#include "rclcpp/rclcpp.hpp"
#include "topic_demo/msg/demo.hpp"

class SubscriberNode : public rclcpp::Node {
public:
  SubscriberNode() : Node("subscriber_node") {
    subscriber_ = this->create_subscription<topic_demo::msg::Demo>(
      "demo_topic", 10,
      std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SubscriberNode started, listening on demo_topic");
  }

private:
  void topic_callback(const topic_demo::msg::Demo &msg) {
    RCLCPP_INFO(this->get_logger(),
      "Received: text='%s' count=%d value=%.1f",
      msg.text.c_str(), msg.count, msg.value);
  }

  rclcpp::Subscription<topic_demo::msg::Demo>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
