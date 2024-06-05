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
    
    gripper_->connect();
    gripper_->activate();

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

    for (const auto& stage : msg->stages)
    {
      RCLCPP_INFO(this->get_logger(), "  - %s", stage.name.c_str());
    }

    processTask(*msg);
  }

  void processTask(const master_project_msgs::msg::Task& task)
  {
    for (const auto& stage : task.stages)
    {
      RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());
      
      // Get the number of waypoints in the current stage
      size_t num_waypoints = stage.waypoints.size();
      RCLCPP_INFO(this->get_logger(), "Number of waypoints in stage %s: %zu", stage.name.c_str(), num_waypoints);
      if (num_waypoints == 0){
        RCLCPP_INFO(this->get_logger(), "Stage %s has no waypoints. Skipping...", stage.name.c_str());
        std::this_thread::sleep_for(200ms);
        continue;
      }
      std::vector<std::vector<double>> robot_path;
      double initial_gripper_position;
      double final_gripper_position;
      double counter_waypoints = 0;
      double finger_change = 0.05;

      for (const auto& waypoint : stage.waypoints)
      {
        std::vector<double> joint_positions(6, 0.0);

        double counter_joints = 0;

        for (const auto& joint : waypoint.joints)
        {
          if (counter_waypoints == num_waypoints-1 && counter_joints == 6){
            final_gripper_position = joint.position;
          }
          if (counter_joints > 6){
            continue;
          }
          else if (counter_joints == 6 && counter_waypoints == 0){
            initial_gripper_position = joint.position;
          }
          else {
            joint_positions[counter_joints] = joint.position;
          } 

          counter_joints ++;  
          std::vector<double> one_pos(joint_positions.begin(), joint_positions.end());
          one_pos.push_back(1.0); //velocity
          one_pos.push_back(1.2); //acceleration
          one_pos.push_back(0.005); //blend (5mm tolerance)
          robot_path.push_back(one_pos);
        }
        counter_waypoints ++;
      }

      if (abs(initial_gripper_position - final_gripper_position) > finger_change){
        gripper_->move(final_gripper_position * 100);
        RCLCPP_INFO(this->get_logger(), "Gripper position set to: %f", final_gripper_position * 100);
      } 
      else{
        rtde_control_->moveJ(robot_path, true);
        std::this_thread::sleep_for(200ms);
      }

      std::this_thread::sleep_for(200ms);
    }
  }

  std::string ip_;
  std::unique_ptr<RTDEControlInterface> rtde_control_;
  std::unique_ptr<RobotiqGripper> gripper_;
  rclcpp::Subscription<master_project_msgs::msg::Task>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskUsageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
