#include <memory>
#include <functional>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "master_project_msgs/action/move_to_target.hpp"

using MoveToTarget = master_project_msgs::action::MoveToTarget;
using GoalHandleMoveToTarget = rclcpp_action::ClientGoalHandle<MoveToTarget>;

class MoveToTargetClient : public rclcpp::Node {
public:
  MoveToTargetClient() : Node("move_to_target_client") {
    this->client_ = rclcpp_action::create_client<MoveToTarget>(this, "move_to_target");
    this->get_user_input();
  }

private:
  rclcpp_action::Client<MoveToTarget>::SharedPtr client_;
  std::string stage_name_;

  void get_user_input() {
    std::cout << "Enter the name of the stage: ";
    std::getline(std::cin, stage_name_);
    send_goal();
  }

  void send_goal() {
    auto goal_msg = MoveToTarget::Goal();
    goal_msg.stage_name = stage_name_; // Set the desired stage name

    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<MoveToTarget>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveToTargetClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveToTargetClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&MoveToTargetClient::result_callback, this, std::placeholders::_1);

    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleMoveToTarget::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleMoveToTarget::SharedPtr,
                         const std::shared_ptr<const MoveToTarget::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: progress = %.2f", feedback->progress);
  }

  void result_callback(const GoalHandleMoveToTarget::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Result: success = %d", result.result->success);
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToTargetClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
