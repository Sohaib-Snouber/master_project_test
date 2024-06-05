from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder("robot").to_dict()

    # Planning Scene Node
    planning_scene_node = Node(
        package='master_project_test',
        executable='planning_scene_node',
        name='planning_scene_node',
        output='screen',
        parameters=[moveit_config]
    )

    # Target Pose Node
    target_pose_node = Node(
        package='master_project_test',
        executable='target_pose_node',
        name='target_pose_node',
        output='screen'
    )

    # Task Execution Node
    task_execution_node = Node(
        package='master_project_test',
        executable='task_execution_node',
        name='task_execution_node',
        output='screen',
        parameters=[moveit_config]
    )

    # Task Print Node
    task_print_node = Node(
        package='master_project_test',
        executable='task_usage_node',
        name='task_usage_node',
        output='screen'
    )

    return LaunchDescription([
        planning_scene_node,
        target_pose_node,
        task_execution_node,
        task_print_node
    ])

if __name__ == '__main__':
    generate_launch_description()
