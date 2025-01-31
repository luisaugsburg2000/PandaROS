#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <memory>
#include <thread>
#include <iostream>
#include <future>
#include <action_msgs/msg/goal_status.hpp>
#include <random>

// Global or static variable to hold the human render mode
static bool human_render_mode = true;

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

// Function to reset the robot to its origin position
void resetRobot(
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const rclcpp::Logger &logger)
{
  RCLCPP_INFO(logger, "Reset command received. Snapping robot to origin position...");

  // Define the origin joint positions (adjust these for your robot)
  std::vector<double> origin_positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}; // Replace with actual origin positions
  auto joint_names = move_group_interface.getJointNames();

  // Publish the joint state to snap the robot to the origin position
  sensor_msgs::msg::JointState joint_message;
  joint_message.name = joint_names;
  joint_message.position = origin_positions;

  sawyer_pub->publish(joint_message);

  // Synchronize MoveIt!'s internal state with the robot's new position
  move_group_interface.setJointValueTarget(origin_positions);
  move_group_interface.setStartStateToCurrentState();

  RCLCPP_INFO(logger, "Robot snapped to origin position.");
}

// Callback for the "robot_reset" topic
void robotResetCallback(
  const std_msgs::msg::Empty::SharedPtr /*msg*/,
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const rclcpp::Logger &logger)
{
  resetRobot(sawyer_pub, move_group_interface, logger);
}

void stepCallback(
  const geometry_msgs::msg::Pose::SharedPtr msg,
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  rclcpp::Publisher<action_msgs::msg::GoalStatus>::SharedPtr status_pub,
  rclcpp::Node::SharedPtr node,
  const rclcpp::Logger &logger)
{
  action_msgs::msg::GoalStatus status_msg;

  RCLCPP_INFO(logger, "Received sphere pose. Starting planning...");
  RCLCPP_INFO(logger, "Received sphere pose: x=%f, y=%f, z=%f", msg->position.x, msg->position.y, msg->position.z);

  // Define the original target pose
  geometry_msgs::msg::Pose target_pose = *msg;

  // PHASE 1: Compute Stopover Pose
  tf2::Vector3 stopover_offset(-0.175, 0.0, 0.1);  // Offset in front of the port
  tf2::Vector3 adjtarget_offset(-0.1, 0.0, 0.1);
  tf2::Quaternion orientation;
  tf2::fromMsg(target_pose.orientation, orientation);

  // Rotate the local offset into the global frame
  tf2::Vector3 global_stopover_offset = tf2::quatRotate(orientation, stopover_offset);
  tf2::Vector3 global_adjtarget_offset = tf2::quatRotate(orientation, adjtarget_offset);

  geometry_msgs::msg::Pose stopover_pose = target_pose;
  geometry_msgs::msg::Pose adjtarget_pose = target_pose;

  // Apply the global offset to the position
  stopover_pose.position.x += global_stopover_offset.x();
  stopover_pose.position.y += global_stopover_offset.y();
  stopover_pose.position.z += global_stopover_offset.z();

  adjtarget_pose.position.x += global_adjtarget_offset.x();
  adjtarget_pose.position.y += global_adjtarget_offset.y();
  adjtarget_pose.position.z += global_adjtarget_offset.z();

  RCLCPP_INFO(logger, "Stopover pose: x=%f, y=%f, z=%f",
              stopover_pose.position.x, stopover_pose.position.y, stopover_pose.position.z);
  RCLCPP_INFO(logger, "AdjTarget pose: x=%f, y=%f, z=%f",
              adjtarget_pose.position.x, adjtarget_pose.position.y, adjtarget_pose.position.z);

  // Add rotation offset to stopover pose
  tf2::Quaternion offset_quaternion;
  offset_quaternion.setRPY(M_PI, 0.0, -(M_PI / 4));  // Roll, Pitch, Yaw offsets
  tf2::Quaternion new_orientation = orientation * offset_quaternion;
  new_orientation.normalize();
  stopover_pose.orientation = tf2::toMsg(new_orientation);
  adjtarget_pose.orientation = tf2::toMsg(new_orientation);

  // PHASE 2: Plan Waypoints (Stopover + Target)
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(stopover_pose);  // Stopover as the first waypoint
  waypoints.push_back(adjtarget_pose); // Final target as the second waypoint

  // Plan the Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;   // Disable jump threshold
  const double eef_step = 0.01;        // EEF step size for interpolation
  double fraction = move_group_interface.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  // Generate a unique goal ID for status messages
  std::random_device rd;
  std::uniform_int_distribution<int> dist(0, 255);
  for (auto &byte : status_msg.goal_info.goal_id.uuid)
  {
    byte = static_cast<uint8_t>(dist(rd));
  }

  // ---------------------------------------------------
  // Check the rendering mode (human_render_mode)
  // ---------------------------------------------------
  if (!human_render_mode)
  {
    // If human_render_mode == false,
    // 1) Do NOT publish the entire trajectory in a thread.
    // 2) Immediately publish only the final pose.
    // 3) Publish status_msg with a value of 4.
    RCLCPP_INFO(logger, "Human render mode disabled: skipping full trajectory publication.");

    if (trajectory.joint_trajectory.points.empty())
    {
      // No valid trajectory was computed
      RCLCPP_ERROR(logger, "No valid trajectory found. Publishing status=ABORTED.");
      status_msg.status = action_msgs::msg::GoalStatus::STATUS_ABORTED;
      status_pub->publish(status_msg);
      return;
    }

    // Get the final point in the computed Cartesian path
    const auto &final_point = trajectory.joint_trajectory.points.back();

    // Publish that final point immediately
    sensor_msgs::msg::JointState joint_message;
    joint_message.name = trajectory.joint_trajectory.joint_names;
    joint_message.position = final_point.positions;
    if (!final_point.velocities.empty())
    {
      joint_message.velocity = final_point.velocities;
    }

    sawyer_pub->publish(joint_message);
    RCLCPP_INFO(logger, "Published final robot pose from Cartesian path immediately.");

    // Now publish the status (set to 4, as requested)
    status_msg.status = 4; // Or any custom code you'd like to indicate "skipped path"
    status_pub->publish(status_msg);
    return; // Done
  }
  // ---------------------------------------------------
  // If human_render_mode == true, do normal behavior
  // ---------------------------------------------------

  if (fraction > 0.95) // If most of the path is valid
  {
    RCLCPP_INFO(logger, "Successfully computed Cartesian path (%.2f%% of path valid)", fraction * 100.0);

    std::promise<void> exitSignal;
    std::future<void> future = exitSignal.get_future();

    // Publish the planned path in a separate thread
    std::thread publisher_thread(
      publishPlannedPath,
      sawyer_pub,
      trajectory.joint_trajectory,
      node,
      logger,
      std::move(exitSignal));

    publisher_thread.detach();
    RCLCPP_INFO(logger, "Executing the Cartesian path...");

    future.wait(); // Wait for execution to complete

    RCLCPP_INFO(logger, "Finished executing the Cartesian path.");
    status_msg.status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to compute a valid Cartesian path (%.2f%% of path valid)", fraction * 100.0);
    status_msg.status = action_msgs::msg::GoalStatus::STATUS_ABORTED;
  }

  // Publish the execution status
  status_pub->publish(status_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create node
  auto node = rclcpp::Node::make_shared("sawyer_robot_controller");
  auto logger = rclcpp::get_logger("move_program");

  // Publisher for joint commands
  auto sawyer_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

  // Publisher for execution status
  auto status_pub = node->create_publisher<action_msgs::msg::GoalStatus>("execution_status", 10);

  // Initialize MoveIt! interface
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");

  // Subscriber for sphere poses
  auto step_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "/step", 10,
    [&move_group_interface, &sawyer_pub, &status_pub, node, logger](const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      stepCallback(msg, move_group_interface, sawyer_pub, status_pub, node, logger);
    });

  // Subscriber for robot reset
  auto reset_sub = node->create_subscription<std_msgs::msg::Empty>(
    "/reset", 10,
    [&sawyer_pub, &move_group_interface, logger](const std_msgs::msg::Empty::SharedPtr msg)
    {
      robotResetCallback(msg, sawyer_pub, move_group_interface, logger);
    });

  // Subscriber for human_render_mode
  // This will update the global/static "human_render_mode" boolean.
  auto human_render_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/human_render_mode", 10,
    [](const std_msgs::msg::Bool::SharedPtr msg)
    {
      human_render_mode = msg->data;
      RCLCPP_INFO(rclcpp::get_logger("move_program"), 
                  "Received /human_render_mode update: %s",
                  human_render_mode ? "true" : "false");
    }
  );

  RCLCPP_INFO(logger, "Waiting for step message on topic '/step'...");
  RCLCPP_INFO(logger, "Waiting for reset commands on topic '/reset'...");
  RCLCPP_INFO(logger, "Waiting for boolean on topic '/human_render_mode' (default = true)...");

  // Spin node
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
