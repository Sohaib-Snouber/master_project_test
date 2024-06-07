#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "master_project_msgs/action/current_state.hpp"
#include "master_project_msgs/action/move_to_target.hpp"
#include "master_project_msgs/action/move_slide.hpp"
#include "master_project_msgs/action/close_gripper.hpp"
#include "master_project_msgs/action/lift_gripper.hpp"
#include "master_project_msgs/action/open_gripper.hpp"
#include "master_project_msgs/msg/task.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/robotiq_gripper.h>

using namespace ur_rtde;
using namespace std::chrono_literals;

class TaskActionServer : public rclcpp::Node {
public:
  using CurrentState = master_project_msgs::action::CurrentState;
  using MoveToTarget = master_project_msgs::action::MoveToTarget;
  using MoveSlide = master_project_msgs::action::MoveSlide;
  using CloseGripper = master_project_msgs::action::CloseGripper;
  using LiftGripper = master_project_msgs::action::LiftGripper;
  using OpenGripper = master_project_msgs::action::OpenGripper;
  
  using Task = master_project_msgs::msg::Task;
  using Stage = master_project_msgs::msg::Stage;
  using Waypoint = master_project_msgs::msg::Waypoint;

  TaskActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("task_action_server", options), ip_("10.130.1.100") {
    rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
    gripper_ = std::make_unique<RobotiqGripper>(ip_);

    RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
    gripper_->connect();
    RCLCPP_INFO(this->get_logger(), "Gripper connected.");
    gripper_->activate();
    RCLCPP_INFO(this->get_logger(), "Gripper activated.");

    task_details_subscriber_ = this->create_subscription<master_project_msgs::msg::Task>(
        "task_details", 10, std::bind(&TaskActionServer::taskDetailsCallback, this, std::placeholders::_1));

    current_state_server_ = rclcpp_action::create_server<CurrentState>(
        this, "current_state", 
        std::bind(&TaskActionServer::handleGoal<CurrentState>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel<CurrentState>, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted<CurrentState>, this, std::placeholders::_1));

    move_to_target_server_ = rclcpp_action::create_server<MoveToTarget>(
        this, "move_to_target",
        std::bind(&TaskActionServer::handleGoal<MoveToTarget>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel<MoveToTarget>, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted<MoveToTarget>, this, std::placeholders::_1));

    move_slide_server_ = rclcpp_action::create_server<MoveSlide>(
        this, "move_slide",
        std::bind(&TaskActionServer::handleGoal<MoveSlide>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel<MoveSlide>, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted<MoveSlide>, this, std::placeholders::_1));

    close_gripper_server_ = rclcpp_action::create_server<CloseGripper>(
        this, "close_gripper",
        std::bind(&TaskActionServer::handleGoal<CloseGripper>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel<CloseGripper>, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted<CloseGripper>, this, std::placeholders::_1));

    lift_gripper_server_ = rclcpp_action::create_server<LiftGripper>(
        this, "lift_gripper",
        std::bind(&TaskActionServer::handleGoal<LiftGripper>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel<LiftGripper>, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted<LiftGripper>, this, std::placeholders::_1));

    open_gripper_server_ = rclcpp_action::create_server<OpenGripper>(
        this, "open_gripper",
        std::bind(&TaskActionServer::handleGoal<OpenGripper>, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel<OpenGripper>, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted<OpenGripper>, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<master_project_msgs::msg::Task>::SharedPtr task_details_subscriber_;
  rclcpp_action::Server<CurrentState>::SharedPtr current_state_server_;
  rclcpp_action::Server<MoveToTarget>::SharedPtr move_to_target_server_;
  rclcpp_action::Server<MoveSlide>::SharedPtr move_slide_server_;
  rclcpp_action::Server<CloseGripper>::SharedPtr close_gripper_server_;
  rclcpp_action::Server<LiftGripper>::SharedPtr lift_gripper_server_;
  rclcpp_action::Server<OpenGripper>::SharedPtr open_gripper_server_;
  
  std::string ip_;
  std::unique_ptr<RTDEControlInterface> rtde_control_;
  std::unique_ptr<RobotiqGripper> gripper_;
  bool task_done_;
  std::unordered_map<std::string, Stage> stages_map_;

  void taskDetailsCallback(const master_project_msgs::msg::Task::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->name.c_str());
    for (const auto & stage : msg->stages) {
      stages_map_[stage.name] = stage;
    }
  }

  template<typename ActionType>
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid, 
      std::shared_ptr<const typename ActionType::Goal> goal) {
    (void)uuid;  // Suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    if (stages_map_.find(goal->stage_name) == stages_map_.end()) {
      RCLCPP_WARN(this->get_logger(), "Stage %s not found", goal->stage_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  template<typename ActionType>
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
    (void)goal_handle;  // Suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  template<typename ActionType>
  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
    std::thread{std::bind(&TaskActionServer::execute<ActionType>, this, goal_handle)}.detach();
  }

  template<typename ActionType>
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    const auto &stage = stages_map_[goal->stage_name];

    processStage(stage);

    auto result = std::make_shared<typename ActionType::Result>();
    result->success = true;
    goal_handle->succeed(result);
  }

  void processStage(const Stage& stage) {
    RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());

    if (stage.waypoints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Stage %s has no waypoints. Skipping...", stage.name.c_str());
      return;
    }

    std::vector<std::vector<double>> robot_path;
    double initial_gripper_position = -1;
    double final_gripper_position = -1;
    double finger_change = 0.1;

    for (size_t i = 0; i < stage.waypoints.size(); ++i) {
      const auto &waypoint = stage.waypoints[i];
      std::vector<double> joint_positions(6, 0.0);

      size_t joint_index = 0;
      for (const auto &joint : waypoint.joints) {
        if (joint_index < 6) {
          joint_positions[joint_index] = joint.position;
        } else if (joint_index == 6) {
          if (i == 0) {
            initial_gripper_position = map_finger_joint_to_gripper(joint.position);
          }
          if (i == stage.waypoints.size() - 1) {
            final_gripper_position = map_finger_joint_to_gripper(joint.position);
          }
        }
        joint_index++;
      }

      joint_positions.push_back(1.0);  // velocity
      joint_positions.push_back(1.2);  // acceleration
      joint_positions.push_back(0.005);  // blend (5mm tolerance)
      robot_path.push_back(joint_positions);
    }

    if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
      RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
      gripper_->move(std::clamp(final_gripper_position, 0.0, 1.0));
      RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
    } else {
      rtde_control_->moveJ(robot_path, true);
    }
  }

  double map_finger_joint_to_gripper(double position) {
    // Implement your own mapping here
    return position;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
