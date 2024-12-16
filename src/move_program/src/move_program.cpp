#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <memory>
#include <thread>

// Function to publish joint commands from the planned trajectory
void publishPlannedPath(
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  const trajectory_msgs::msg::JointTrajectory &trajectory,
  rclcpp::Node::SharedPtr node,
  const rclcpp::Logger &logger)
{
  RCLCPP_INFO(logger, "Publishing joint commands from planned trajectory...");

  auto start_time = node->now();

  for (const auto &point : trajectory.points)
  {
    // Sleep until the next point
    auto now = node->now();
    auto target_time = start_time + rclcpp::Duration::from_seconds(
                                       point.time_from_start.sec +
                                       point.time_from_start.nanosec * 1e-9);
    auto sleep_time = (target_time - now).to_chrono<std::chrono::nanoseconds>();

    if (sleep_time.count() > 0)
    {
      std::this_thread::sleep_for(sleep_time);
    }

    // Publish joint positions
    sensor_msgs::msg::JointState joint_message;
    joint_message.name = trajectory.joint_names;
    joint_message.position = point.positions;

    if (!point.velocities.empty())
    {
      joint_message.velocity = point.velocities;
    }

    sawyer_pub->publish(joint_message);

    RCLCPP_INFO(logger, "Published joint positions.");
  }
  RCLCPP_INFO(logger, "Finished publishing planned trajectory.");
}

void spherePoseCallback(
  const geometry_msgs::msg::Pose::SharedPtr msg,
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  rclcpp::Node::SharedPtr node,
  const rclcpp::Logger &logger)
{
  RCLCPP_INFO(logger, "Received sphere pose. Starting planning...");
  RCLCPP_INFO(logger, "Received sphere pose: x=%f, y=%f, z=%f", msg->position.x, msg->position.y, msg->position.z);

  // Set the received pose as the goal
  move_group_interface.setPoseTarget(*msg);

  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "Planning successful. Executing the plan...");

    // Publish the planned path in a separate thread
    std::thread publisher_thread(
      publishPlannedPath,
      sawyer_pub,
      plan.trajectory_.joint_trajectory,
      node,
      logger);

    publisher_thread.detach();
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create node
  auto node = rclcpp::Node::make_shared("sawyer_robot_controller");
  auto logger = rclcpp::get_logger("move_program");

  // Publisher for joint commands
  auto sawyer_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

  // Initialize MoveIt! interface
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");

  // Subscriber for sphere poses
  auto sphere_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "sphere_pose", 10,
    [&move_group_interface, &sawyer_pub, node, logger](const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      spherePoseCallback(msg, move_group_interface, sawyer_pub, node, logger);
    });

  RCLCPP_INFO(logger, "Waiting for sphere pose messages on topic '/sphere_pose'...");

  // Spin node
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
