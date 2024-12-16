#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <memory>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

// Publish joint commands from the planned trajectory
void publishPlannedPath(
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sawyer_pub,
  const trajectory_msgs::msg::JointTrajectory &trajectory,
  rclcpp::Node::SharedPtr node,
  const rclcpp::Logger &logger)
{
  RCLCPP_INFO(logger, "Publishing joint commands from planned trajectory...");

  // Get the start time
  auto start_time = node->now();

  for (const auto &point : trajectory.points)
  {
    // Calculate the time to wait until the next point
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

    RCLCPP_INFO(logger, "Published joint positions: [%s]",
                std::accumulate(point.positions.begin(), point.positions.end(), std::string(),
                                [](const std::string &a, double b)
                                { return a + (a.empty() ? "" : ", ") + std::to_string(b); })
                    .c_str());
  }

  RCLCPP_INFO(logger, "Finished publishing planned trajectory.");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sawyer_robot");

  // Initialize publisher
  auto sawyer_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

  // Control variables
  const rclcpp::Logger logger = rclcpp::get_logger("move_program");

  // Execute MoveIt! planning
  moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "panda_arm");

  // Define target pose
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -3.14 / 2);
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  geometry_msgs::msg::Pose GoalPose;
  GoalPose.orientation = msg_quat;
  GoalPose.position.x = 0.3;
  GoalPose.position.y = -0.3;
  GoalPose.position.z = 0.4;

  MoveGroupInterface.setPoseTarget(GoalPose);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  auto const outcome = static_cast<bool>(MoveGroupInterface.plan(plan1));

  if (outcome)
  {
    RCLCPP_INFO(logger, "Planning successful.");

    // Run trajectory publisher in a separate thread
    std::thread publisher_thread(
      publishPlannedPath,
      sawyer_pub,
      plan1.trajectory_.joint_trajectory,
      node,
      logger);

    // Spin node for other callbacks
    rclcpp::spin(node);

    // Wait for publisher thread to finish
    if (publisher_thread.joinable())
    {
      publisher_thread.join();
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
