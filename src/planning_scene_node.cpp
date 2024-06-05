#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlanningSceneNode : public rclcpp::Node
{
public:
  PlanningSceneNode();

private:
  void setupPlanningScene();
  moveit_msgs::msg::CollisionObject createCollisionObject(
    const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose);
  moveit_msgs::msg::ObjectColor createObjectColor(
    const std::string& id, float r, float g, float b, float a);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

PlanningSceneNode::PlanningSceneNode() : Node("planning_scene_node")
{
  setupPlanningScene();
}

void PlanningSceneNode::setupPlanningScene()
{
  /* // Create collision objects
  shape_msgs::msg::SolidPrimitive primitive_table1;
  primitive_table1.type = primitive_table1.BOX;
  primitive_table1.dimensions = {0.9, 0.6, 0.2};
  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.45;
  table1_pose.position.y = -0.6;
  table1_pose.position.z = 0.1;
  moveit_msgs::msg::CollisionObject collision_object_table1 = createCollisionObject("table1", primitive_table1, table1_pose);

  shape_msgs::msg::SolidPrimitive primitive_table2;
  primitive_table2.type = primitive_table2.BOX;
  primitive_table2.dimensions = {0.6, 0.9, 0.2};
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = -0.60;
  table2_pose.position.y = 0.0;
  table2_pose.position.z = 0.1;
  moveit_msgs::msg::CollisionObject collision_object_table2 = createCollisionObject("table2", primitive_table2, table2_pose);

  shape_msgs::msg::SolidPrimitive primitive_target1;
  primitive_target1.type = primitive_target1.CYLINDER;
  primitive_target1.dimensions = {0.1, 0.02}; // height, radius
  geometry_msgs::msg::Pose target1_pose;
  target1_pose.orientation.w = 1.0;
  target1_pose.position.x = 0.45;
  target1_pose.position.y = -0.35;
  target1_pose.position.z = 0.25;
  moveit_msgs::msg::CollisionObject collision_object_target1 = createCollisionObject("target1", primitive_target1, target1_pose);
 */
  shape_msgs::msg::SolidPrimitive primitive_surface;
  primitive_surface.type = primitive_surface.BOX;
  primitive_surface.dimensions = {0.75, 0.8, 0.1};
  geometry_msgs::msg::Pose surface_pose;
  surface_pose.orientation.w = 1.0;
  surface_pose.position.x = 0.16;
  surface_pose.position.y = 0.25;
  surface_pose.position.z = -0.05;
  moveit_msgs::msg::CollisionObject collision_object_surface = createCollisionObject("surface", primitive_surface, surface_pose);

  /* // Create object colors
  moveit_msgs::msg::ObjectColor table1_color = createObjectColor("table1", 1.0, 0.5, 0.5, 1.0);
  moveit_msgs::msg::ObjectColor table2_color = createObjectColor("table2", 1.0, 0.5, 0.5, 1.0);
  moveit_msgs::msg::ObjectColor target1_color = createObjectColor("target1", 0.5, 0.0, 1.0, 1.0); */
  moveit_msgs::msg::ObjectColor surface_color = createObjectColor("surface", 0.5, 0.5, 0.5, 1.0);

  // Create a PlanningScene message and add the collision objects and their colors
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects = {collision_object_surface};
  planning_scene_msg.object_colors = {surface_color};
  /* planning_scene_msg.world.collision_objects = {collision_object_table1, collision_object_table2, collision_object_target1, collision_object_surface};
  planning_scene_msg.object_colors = {table1_color, table2_color, target1_color, surface_color}; */

  // Apply the planning scene
  planning_scene_interface_.applyPlanningScene(planning_scene_msg);

  RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
}

moveit_msgs::msg::CollisionObject PlanningSceneNode::createCollisionObject(
  const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = id;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;
  return collision_object;
}

moveit_msgs::msg::ObjectColor PlanningSceneNode::createObjectColor(
  const std::string& id, float r, float g, float b, float a)
{
  moveit_msgs::msg::ObjectColor color;
  color.id = id;
  color.color.r = r;
  color.color.g = g;
  color.color.b = b;
  color.color.a = a;
  return color;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
