#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TaskPrintNode : public rclcpp::Node
{
public:
  TaskPrintNode() : Node("task_print_node")
  {
    task_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/demo_task", 10, std::bind(&TaskPrintNode::taskCallback, this, std::placeholders::_1));
  }

private:
  void taskCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received task: %s", msg->data.c_str());
    // Print task details or handle the task message
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_subscriber_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskPrintNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
