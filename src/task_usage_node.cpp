#include <rclcpp/rclcpp.hpp>
#include <master_project_msgs/msg/task.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include <vector>

using namespace ur_rtde;
using namespace std::chrono_literals;

class TaskUsageNode : public rclcpp::Node
{
public:
  TaskUsageNode()
    : Node("task_usage_node"), ip_("10.130.1.100")
  {
    rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
    gripper_ = std::make_unique<RobotiqGripper>(ip_);
    
    // Debug: Print statements for gripper connection and activation
    RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
    gripper_->connect();
    RCLCPP_INFO(this->get_logger(), "Gripper connected.");
    gripper_->activate();
    RCLCPP_INFO(this->get_logger(), "Gripper activated.");

    subscription_ = this->create_subscription<master_project_msgs::msg::Task>(
      "/task_details", 10, std::bind(&TaskUsageNode::taskDetailsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TaskUsageNode has been started.");
  }

private:
  void taskDetailsCallback(const master_project_msgs::msg::Task::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received TaskDetails message: %s", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "Number of stages: %u", msg->number_of_stages);
    RCLCPP_INFO(this->get_logger(), "Stages:");
    
    processTask(*msg);
  }

  void processTask(const master_project_msgs::msg::Task& task)
  {
    for (const auto& stage : task.stages)
    {
      RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());
      
      RCLCPP_INFO(this->get_logger(), "Number of waypoints in stage %s: %zu", stage.name.c_str(), stage.waypoints.size());
      if (stage.waypoints.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Stage %s has no waypoints. Skipping...", stage.name.c_str());
        std::this_thread::sleep_for(50ms);
        continue;
      }

      std::vector<std::vector<double>> robot_path;
      double initial_gripper_position = -1;
      double final_gripper_position = -1;
      double finger_change = 0.1;

      for (size_t i = 0; i < stage.waypoints.size(); ++i)
      {
        const auto& waypoint = stage.waypoints[i];
        std::vector<double> joint_positions(6, 0.0);  // Ensure this only has 6 elements for the robot arm joints

        size_t joint_index = 0;
        for (const auto& joint : waypoint.joints)
        {
          if (joint_index < 6) {  // Only handle the first 6 joints for the robot arm
            joint_positions[joint_index] = joint.position;
          } else if (joint_index == 6) {  // Handle the gripper position
            if (i == stage.waypoints.size() - 1) {
              final_gripper_position = map_finger_joint_to_gripper(joint.position);
            }
            if (i == 0) {
              initial_gripper_position = map_finger_joint_to_gripper(joint.position);
            }
          }
          joint_index++;
        }

        std::vector<double> one_pos(joint_positions.begin(), joint_positions.end());
        one_pos.push_back(1.0);  // velocity
        one_pos.push_back(1.2);  // acceleration
        one_pos.push_back(0.005);  // blend (5mm tolerance)
        robot_path.push_back(one_pos);
      }

      // Debug: Print gripper positions
      RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f", initial_gripper_position);
      RCLCPP_INFO(this->get_logger(), "Final gripper position: %f", final_gripper_position);

      if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
        // Debug: Before moving the gripper
        RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
        final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
        RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
        gripper_->move(final_gripper_position);
        // Debug: After moving the gripper
        RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
      } else {
        rtde_control_->moveJ(robot_path, true);
      }

      std::this_thread::sleep_for(50ms);
    }
    task_done_ = true;
  }

  double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  }

  double map_finger_joint_to_gripper(double finger_joint_position) {
    double close_finger_joint = 0.69991196175;
    double open_finger_joint = 0.07397215645645;
    return map_value(finger_joint_position, open_finger_joint, close_finger_joint, 0.988235, 0.105882);
  }

  std::string ip_;
  std::unique_ptr<RTDEControlInterface> rtde_control_;
  std::unique_ptr<RobotiqGripper> gripper_;
  rclcpp::Subscription<master_project_msgs::msg::Task>::SharedPtr subscription_;
  bool task_done_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskUsageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
