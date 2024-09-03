#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


namespace planning {
class AStarNode : rclcpp::Node {
public:
  AStarNode() : rclcpp::Node("a_star") {
    auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_settings.best_effort();
    qos_settings.reliable();
    qos_settings.transient_local();
    status_receiver = this->create_subscription<std_msgs::msg::String>(
        "in_status", qos_settings,
        std::bind(&AStarNode::status_callback, this, std::placeholders::_1));
    status_publisher =
        this->create_publisher<std_msgs::msg::String>("status", qos_settings);
  }

private:
  void status_callback(const std_msgs::msg::String &msg) {
    RCLCPP_INFO(this->get_logger(), "Message", msg.data.c_str());
  }

  void generate_path();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_receiver;
};

} // namespace planning