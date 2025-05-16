#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloWorldNode : public rclcpp::Node
{
public:
  HelloWorldNode()
    : Node("hello_world_node")
  {
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, World!";
      RCLCPP_INFO(get_logger(), "Publishing: %s", message.data.c_str());
      publisher_->publish(message);
    };

    // Publish "Hello, World!" every second
    timer_ = create_wall_timer(std::chrono::seconds(1), timer_callback);
    publisher_ = create_publisher<std_msgs::msg::String>("hello_world", 10);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorldNode>());
  rclcpp::shutdown();
  return 0;
}