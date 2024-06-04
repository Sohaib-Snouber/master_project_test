#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode : public rclcpp::Node
{
public:
  MTCTaskNode();
  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task printTaskDetails(const mtc::Task& task);
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  geometry_msgs::msg::PoseStamped current_target_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;
  mtc::Task task_;
  bool new_pose_received_ = false; // Flag to check if a new pose is received
  int task_counter_ = 0; // Counter to keep track of task numbers
  moveit::core::RobotModelPtr robot_model_; // Robot model


};

MTCTaskNode::MTCTaskNode() : Node("mtc_node")
{
  target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, std::bind(&MTCTaskNode::targetPoseCallback, this, std::placeholders::_1));
  // Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
  robot_model_ = robot_model_loader.getModel();
  if (!robot_model_)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load robot model");
    throw std::runtime_error("Failed to load robot model");
  }
}

void MTCTaskNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_target_pose_ = *msg;
  new_pose_received_ = true; // Set flag to true on new pose reception
  RCLCPP_INFO(this->get_logger(), "New target pose received: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  setupPlanningScene();  // Ensure the planning scene is set up during node initialization
  doTask(); // Call doTask directly from the callback}
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject collision_object_table1;
  collision_object_table1.header.frame_id = "world";
  collision_object_table1.id = "table1";
  shape_msgs::msg::SolidPrimitive primitive_table1;
  primitive_table1.type = primitive_table1.BOX;
  primitive_table1.dimensions.resize(3);
  primitive_table1.dimensions[primitive_table1.BOX_X] = 0.9;
  primitive_table1.dimensions[primitive_table1.BOX_Y] = 0.6;
  primitive_table1.dimensions[primitive_table1.BOX_Z] = 0.2;
  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.45;
  table1_pose.position.y = -0.6;
  table1_pose.position.z = 0.1;
  collision_object_table1.primitives.push_back(primitive_table1);
  collision_object_table1.primitive_poses.push_back(table1_pose);
  collision_object_table1.operation = collision_object_table1.ADD;

  moveit_msgs::msg::CollisionObject collision_object_table2;
  collision_object_table2.header.frame_id = "world";
  collision_object_table2.id = "table2";
  shape_msgs::msg::SolidPrimitive primitive_table2;
  primitive_table2.type = primitive_table2.BOX;
  primitive_table2.dimensions.resize(3);
  primitive_table2.dimensions[primitive_table2.BOX_X] = 0.6;
  primitive_table2.dimensions[primitive_table2.BOX_Y] = 0.9;
  primitive_table2.dimensions[primitive_table2.BOX_Z] = 0.2;
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = -0.60;
  table2_pose.position.y = 0.0;
  table2_pose.position.z = 0.1;
  collision_object_table2.primitives.push_back(primitive_table2);
  collision_object_table2.primitive_poses.push_back(table2_pose);
  collision_object_table2.operation = collision_object_table2.ADD;

  moveit_msgs::msg::CollisionObject collision_object_target1;
  collision_object_target1.header.frame_id = "world";
  collision_object_target1.id = "target1";
  shape_msgs::msg::SolidPrimitive primitive_target1;
  primitive_target1.type = primitive_target1.CYLINDER;
  primitive_target1.dimensions.resize(2);
  primitive_target1.dimensions[0] = 0.1; // height
  primitive_target1.dimensions[1] = 0.02; // radius
  geometry_msgs::msg::Pose target1_pose;
  target1_pose.orientation.w = 1.0;
  target1_pose.position.x = 0.45;
  target1_pose.position.y = -0.35;
  target1_pose.position.z = 0.25;
  collision_object_target1.primitives.push_back(primitive_target1);
  collision_object_target1.primitive_poses.push_back(target1_pose);
  collision_object_target1.operation = collision_object_target1.ADD;

  moveit_msgs::msg::CollisionObject collision_object_surface;
  collision_object_surface.header.frame_id = "world";
  collision_object_surface.id = "surface";
  shape_msgs::msg::SolidPrimitive primitive_surface;
  primitive_surface.type = primitive_surface.BOX;
  primitive_surface.dimensions.resize(3);
  primitive_surface.dimensions[primitive_surface.BOX_X] = 0.75;
  primitive_surface.dimensions[primitive_surface.BOX_Y] = 0.8;
  primitive_surface.dimensions[primitive_surface.BOX_Z] = 0.1;
  geometry_msgs::msg::Pose surface_pose;
  surface_pose.orientation.w = 1.0;
  surface_pose.position.x = 0.16;
  surface_pose.position.y = 0.25;
  surface_pose.position.z = -0.05;
  collision_object_surface.primitives.push_back(primitive_surface);
  collision_object_surface.primitive_poses.push_back(surface_pose);
  collision_object_surface.operation = collision_object_surface.ADD;

  RCLCPP_INFO(LOGGER, "collision objects created");

  // Define colors
  moveit_msgs::msg::ObjectColor table1_color;
  table1_color.id = collision_object_table1.id;
  table1_color.color.r = 1.0; // Red
  table1_color.color.g = 0.5; // Green
  table1_color.color.b = 0.5; // Blue
  table1_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor table2_color;
  table2_color.id = collision_object_table2.id;
  table2_color.color.r = 1.0; // Red
  table2_color.color.g = 0.5; // Green
  table2_color.color.b = 0.5; // Blue
  table2_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor target1_color;
  target1_color.id = collision_object_target1.id;
  target1_color.color.r = 0.5; // Red
  target1_color.color.g = 0.0; // Green
  target1_color.color.b = 1.0; // Blue
  target1_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor surface_color;
  surface_color.id = collision_object_surface.id;
  surface_color.color.r = 0.5; // Gray
  surface_color.color.g = 0.5;
  surface_color.color.b = 0.5;
  surface_color.color.a = 1.0; // Alpha

  // Create a PlanningScene message and add the collision object and its color
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects.push_back(collision_object_table1);
  planning_scene_msg.world.collision_objects.push_back(collision_object_table2);
  planning_scene_msg.world.collision_objects.push_back(collision_object_target1);
  planning_scene_msg.world.collision_objects.push_back(collision_object_surface);
  planning_scene_msg.object_colors.push_back(table1_color);
  planning_scene_msg.object_colors.push_back(table2_color);
  planning_scene_msg.object_colors.push_back(target1_color);
  planning_scene_msg.object_colors.push_back(surface_color);

  // Apply the planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyPlanningScene(planning_scene_msg);

  // Create a RobotModelLoader
  robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(robot_model);

  // Modify the ACM to allow collision between base_link_inertia and surface
  collision_detection::AllowedCollisionMatrix& acm = planning_scene.getAllowedCollisionMatrixNonConst();
  acm.setEntry("base_link_inertia", "surface", true);

  // Update the planning scene with the modified ACM
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
  acm.getMessage(acm_msg);
  planning_scene_msg.allowed_collision_matrix = acm_msg;
  planning_scene_interface.applyPlanningScene(planning_scene_msg);

  RCLCPP_INFO(LOGGER, "allow collision between base_link_inertia and surface.");
}

void MTCTaskNode::doTask()
{
  if (!new_pose_received_)
    return; // Return if no new pose has been received
  task_counter_++;  // Increment the task counter
  createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Task execution failed");
    return;
  }

  printTaskDetails(task_);
  new_pose_received_ = false; // Reset the flag after processing
  return;
}

mtc::Task MTCTaskNode::printTaskDetails(const mtc::Task& task)
{
    RCLCPP_INFO(LOGGER, "Task stages:");

    const auto* container = task.stages();
    if (container)
    {
        // Traverse the children stages and print their details
        container->traverseChildren([&](const mtc::Stage& stage, unsigned int depth) -> bool {
            RCLCPP_INFO(LOGGER, "Stage: %s", stage.name().c_str());

            // If the stage has solutions, print the details
            if (!stage.solutions().empty())
            {
                for (const auto& solution : stage.solutions())
                {
                    RCLCPP_INFO(LOGGER, "  Solution Comment: %s", solution->comment().c_str());

                    // Print trajectory details if available
                    const auto* sub_trajectory = dynamic_cast<const mtc::SubTrajectory*>(solution.get());
                    if (sub_trajectory && sub_trajectory->trajectory())
                    {
                        const auto& trajectory = *sub_trajectory->trajectory();
                        RCLCPP_INFO(LOGGER, "  Trajectory has %zu waypoints", trajectory.getWayPointCount());
                        for (size_t j = 0; j < trajectory.getWayPointCount(); ++j)
                        {
                            const auto& waypoint = trajectory.getWayPoint(j);
                            std::stringstream ss;
                            ss << "  Waypoint " << j << ": ";
                            for (const auto& joint : waypoint.getVariableNames())
                            {
                                ss << joint << "=" << waypoint.getVariablePosition(joint) << " ";
                            }
                            RCLCPP_INFO(LOGGER, "%s", ss.str().c_str());
                        }
                    }
                }
            }
            else
            {
                RCLCPP_INFO(LOGGER, "  No solutions for this stage");
            }
            return true; // Continue traversal
        });
    }
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    std::string task_name = "demo task " + std::to_string(task_counter_);
    task_.stages()->setName(task_name);
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

    // Define the slide pose
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
    task.add(std::move(reenable_collision_target1_table1));

    return task;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MTCTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
