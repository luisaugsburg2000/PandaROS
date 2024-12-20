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

#include <iostream>
#include <future>

// Function to publish joint commands from the planned trajectory
void publishPlannedPath(
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  const trajectory_msgs::msg::JointTrajectory &trajectory,
  rclcpp::Node::SharedPtr node,
  const rclcpp::Logger &logger,
  std::promise<void> exitSignal)
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
  exitSignal.set_value();
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

  // PHASE 1 --------------------------------------------------
  // Copy the received pose
  geometry_msgs::msg::Pose target_pose = *msg;

  // Define the offset in local space
  tf2::Vector3 local_offset(-0.175, 0.0, 0.1);  // x -> how far in front of the port

  // Convert the target orientation to tf2::Quaternion
  tf2::Quaternion orientation;
  tf2::fromMsg(target_pose.orientation, orientation);

  // Rotate the local offset to the global frame
  tf2::Vector3 global_offset = tf2::quatRotate(orientation, local_offset);

  // Apply the global offset to the position
  target_pose.position.x += global_offset.x();
  target_pose.position.y += global_offset.y();
  target_pose.position.z += global_offset.z();

  RCLCPP_INFO(logger, "Adjusted position: x=%f, y=%f, z=%f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Add rotation offset
  tf2::Quaternion offset_quaternion;
  double roll_offset = M_PI;    // Add roll offset in radians
  double pitch_offset = 0.0;   // Add pitch offset in radians
  double yaw_offset = -(M_PI / 4);  // Add yaw offset (example: 45 degrees)

  offset_quaternion.setRPY(roll_offset, pitch_offset, yaw_offset);

  // Apply the rotation offset to the current orientation
  tf2::Quaternion new_orientation = orientation * offset_quaternion;
  new_orientation.normalize();

  // Update the pose's orientation with the offset applied
  target_pose.orientation = tf2::toMsg(new_orientation);

  // Set the adjusted pose as the goal
  move_group_interface.setPoseTarget(target_pose);

  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    std::promise<void> exitSignal;
    std::future<void> future = exitSignal.get_future();

    // Publish the planned path in a separate thread
    std::thread publisher_thread(
      publishPlannedPath,
      sawyer_pub,
      plan.trajectory_.joint_trajectory,
      node,
      logger,
      std::move(exitSignal));

    publisher_thread.detach();
    RCLCPP_INFO(logger, "Planning successful. Executing the plan...");

    future.wait();
    RCLCPP_INFO(logger, "AYAYAYAYAYAYAY");
    // EXECUTE PHASE 2 HERE -->
    // local_offset = tf2::Vector3(-0.1, 0.0, 0.1);
    // tf2::Vector3 global_offset = tf2::quatRotate(orientation, local_offset);
    // target_pose.position.x += global_offset.x();
    // target_pose.position.y += global_offset.y();
    // target_pose.position.z += global_offset.z();

    // move_group_interface.setPoseTarget(target_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // if (move_group_interface.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS)
    // {
    //   std::promise<void> exitSignal2;
    //   std::future<void> future2 = exitSignal2.get_future();

    //   // Publish the planned path in a separate thread
    //   std::thread publisher_thread(
    //     publishPlannedPath,
    //     sawyer_pub,
    //     plan.trajectory_.joint_trajectory,
    //     node,
    //     logger,
    //     std::move(exitSignal2));

    //   publisher_thread.detach();
    //   RCLCPP_INFO(logger, "Planning successful. Executing the plan...");

    //   future2.wait();
    //   RCLCPP_INFO(logger, "BZBZBZBZBZBZBZB");
    // }
    // else
    // {
    //   RCLCPP_ERROR(logger, "Planning 2 failed!");
    // }
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
