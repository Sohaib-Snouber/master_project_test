#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TaskUsageNode : public rclcpp::Node
{
public:

  TaskUsageNode();

private:

  void taskCallback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_subscriber_;

};

TaskUsageNode::TaskUsageNode() : Node("task_usage_node")
{
  task_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/demo_task", 10, std::bind(&TaskUsageNode::taskCallback, this, std::placeholders::_1));
}

void TaskUsageNode::taskCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received task: %s", msg->data.c_str());
  // Print task details or handle the task message
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskUsageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
