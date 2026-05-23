#include "rclcpp/rclcpp.hpp"
#include "topic_demo/msg/demo.hpp"

class PublisherNode : public rclcpp::Node {
public:
  PublisherNode() : Node("publisher_node"), count_(0) {
    publisher_ = this->create_publisher<topic_demo::msg::Demo>("demo_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PublisherNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "PublisherNode started");
  }

private:
  void timer_callback() {
    auto msg = topic_demo::msg::Demo();
    msg.text = "Hello from publisher";
    msg.count = count_++;
    msg.value = count_ * 1.5f;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published: text='%s' count=%d value=%.1f",
      msg.text.c_str(), msg.count, msg.value);
  }

  rclcpp::Publisher<topic_demo::msg::Demo>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
