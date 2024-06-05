#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("execution_task");
namespace mtc = moveit::task_constructor;

class TaskExecutionNode : public rclcpp::Node
{
public:
  TaskExecutionNode(const rclcpp::NodeOptions& options);
  void setup();

private:
  void taskPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void createAndPublishTask();
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_publisher_;
  geometry_msgs::msg::PoseStamped current_target_pose_;
  mtc::Task task_;
};

TaskExecutionNode::TaskExecutionNode(const rclcpp::NodeOptions& options) 
  : Node("task_execution_node", options)
{
  // Initialize the subscriber
  target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task_pose", 10, std::bind(&TaskExecutionNode::taskPoseCallback, this, std::placeholders::_1));
  
  // Initialize the publisher
  task_publisher_ = this->create_publisher<std_msgs::msg::String>("/demo_task", 10); 
}
void TaskExecutionNode::setup()
{
  // Create a RobotModelLoader
  robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
}

void TaskExecutionNode::taskPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_target_pose_ = *msg;
  createAndPublishTask();
  // Process the received pose and execute the task
  std_msgs::msg::String task_msg;
  task_msg.data = "Task execution started"; // Example message
  task_publisher_->publish(task_msg);
  RCLCPP_INFO(this->get_logger(), "Task execution started for pose: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}
void TaskExecutionNode::createAndPublishTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(shared_from_this());

  const auto& arm_group_name = "ur5e_arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "flange";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  // Calculate target pose
  geometry_msgs::msg::PoseStamped target_pose = current_target_pose_;
  // Extract quaternion from the target pose
  tf2::Quaternion q(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);

  // Define a local unit vector along the X-axis (forward direction)
  tf2::Vector3 local_vector(1, 0, 0);

  // Rotate the local vector using the quaternion to get the direction in the world frame
  tf2::Vector3 direction = tf2::quatRotate(q, local_vector);

  // Normalize the direction vector to ensure it is a unit vector
  direction.normalize();

  // Scale the direction vector by 10 cm (0.1 meters)
  double offset = 0.1;  // 10 cm
  tf2::Vector3 offset_vector = direction * offset;

  // Create a new pose with the new position and the same orientation
  geometry_msgs::msg::Pose new_pose;
  target_pose.pose.position.x = target_pose.pose.position.x - offset_vector.x();
  target_pose.pose.position.y = target_pose.pose.position.y - offset_vector.y();
  target_pose.pose.position.z = target_pose.pose.position.z - offset_vector.z();
  target_pose.pose.orientation = target_pose.pose.orientation;  // Keep the same orientation

  // Open hand
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Move to target pose
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
  move_to_target->setGroup(arm_group_name);
  move_to_target->setGoal(target_pose);
  task.add(std::move(move_to_target));

  /* // Define the slide pose
  geometry_msgs::msg::PoseStamped slide_pose = current_target_pose_;

  // Move to slide pose
  auto move_to_slide = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide->setGroup(arm_group_name);
  move_to_slide->setGoal(slide_pose);
  task.add(std::move(move_to_slide));

  // Allow collision (hand, target1) temporarily
  auto allow_collision_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, target1)");
  allow_collision_target1->allowCollisions("target1",
                                            task.getRobotModel()
                                                ->getJointModelGroup(hand_group_name)
                                                ->getLinkModelNamesWithCollisionGeometry(),
                                            true);
  task.add(std::move(allow_collision_target1));

  // Allow collision (target1, table1) temporarily
  auto allow_collision_target1_table1 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (target1, table1)");
  allow_collision_target1_table1->allowCollisions("target1", {"table1"}, true);
  task.add(std::move(allow_collision_target1_table1));

  // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task.add(std::move(stage_close_hand));

  // Attach target1 to the gripper
  auto attach_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("attach target1");
  attach_target1->attachObject("target1", hand_frame);
  task.add(std::move(attach_target1));

  // Lift target1 slightly to avoid collision with the table
  auto lift_target1 = std::make_unique<mtc::stages::MoveRelative>("lift target1", cartesian_planner);
  lift_target1->properties().set("marker_ns", "lift_target1");
  lift_target1->properties().set("link", hand_frame);
  lift_target1->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  lift_target1->setMinMaxDistance(0.1, 0.15); // Increase lift height

  // Set lift direction
  geometry_msgs::msg::Vector3Stamped lift_direction;
  lift_direction.header.frame_id = "world";
  lift_direction.vector.x = -direction.x(); // Lift as same as target orientation
  lift_direction.vector.y = -direction.y();
  lift_direction.vector.z = -direction.z();
  lift_target1->setDirection(lift_direction);
  task.add(std::move(lift_target1));

  // Re-enable collision (target1, table1) after lifting
  auto reenable_collision_target1_table1 = std::make_unique<mtc::stages::ModifyPlanningScene>("reenable collision (target1, table1)");
  reenable_collision_target1_table1->allowCollisions("target1", {"table1"}, false);
  task.add(std::move(reenable_collision_target1_table1)); */

// Initialize the task and publish task details
  std_msgs::msg::String task_msg;
  try
  {
    task.init();
    task_msg.data = "Task Initialized: " + task.stages()->name();
  }
  catch (mtc::InitStageException& e)
  {
    task_msg.data = "Task Initialization Failed: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), e.what());
    task_publisher_->publish(task_msg);
    return;
  }
  task_publisher_->publish(task_msg);

  // Plan the task
  if (!task.plan(20))
  {
    task_msg.data = "Task Planning Failed";
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    task_publisher_->publish(task_msg);
    return;
  }
  task_msg.data = "Task Planned Successfully";
  task_publisher_->publish(task_msg);

  // Publish the planned task
  task.introspection().publishSolution(*task.solutions().front());
  task_msg.data = "Task Published: " + task.solutions().front()->comment();
  task_publisher_->publish(task_msg);

  // Execute the task
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    task_msg.data = "Task Execution Failed";
    RCLCPP_ERROR(this->get_logger(), "Task execution failed");
    task_publisher_->publish(task_msg);
    return;
  }
  task_msg.data = "Task Executed Successfully";
  task_publisher_->publish(task_msg);
}
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<TaskExecutionNode>(options);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  });  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
