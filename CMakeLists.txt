cmake_minimum_required(VERSION 3.8)
project(master_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(ur_rtde REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_task_constructor_core_INCLUDE_DIRS}
  ${ur_rtde_INCLUDE_DIRS}
)

add_executable(sync_robot_state_node src/sync_robot_state_node.cpp)
ament_target_dependencies(sync_robot_state_node rclcpp moveit_core moveit_ros_planning_interface moveit_task_constructor_core ur_rtde)

add_executable(target_mtc_node src/target_mtc_node.cpp)
ament_target_dependencies(target_mtc_node 
  moveit_task_constructor_core
  rclcpp
)

install(TARGETS
  sync_robot_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS target_mtc_node
  DESTINATION lib/${PROJECT_NAME})

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${ur_rtde_INCLUDE_DIRS} # Include ur_rtde headers
)
target_link_libraries(sync_robot_state_node
  ur_rtde::rtde
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
