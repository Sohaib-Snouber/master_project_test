#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlanningSceneNode : public rclcpp::Node
{
public:
  PlanningSceneNode() : Node("planning_scene_node")
  {
    setupPlanningScene();
  }

private:
  void setupPlanningScene()
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

    RCLCPP_INFO(this->get_logger(), "Collision objects created");

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

    RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
