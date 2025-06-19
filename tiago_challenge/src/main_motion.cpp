#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"

#include "Motionplanning_arms.hpp"
#include "RobotTaskStatus.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>



// filepath: /home/bal/ros2_ws_sahri/src/tiago_challenge/src/main_motion.cpp
// ...existing code...

// Dedicated callback function for target pose subscription
void target_pose_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg,
  std::shared_ptr<icr_Motionplanning_arms> node)
{
  RCLCPP_INFO(node->get_logger(), "Received target pose from topic.");
  try {
    node->motion_planning_control(*msg, RobotTaskStatus::Arm::ARM_torso);
    RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
    node->GripperControl("CLOSE");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  }
}

// Callback for gripper command subscription
void gripper_cmd_callback(
  const std_msgs::msg::String::SharedPtr msg,
  std::shared_ptr<icr_Motionplanning_arms> node)
{
  RCLCPP_INFO(node->get_logger(), "Received gripper command: '%s'", msg->data.c_str());
  try {
    node->GripperControl(msg->data);
    RCLCPP_INFO(node->get_logger(), "Gripper command executed.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Gripper command failed: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<icr_Motionplanning_arms>();

  node->GripperControl("OPEN");

  // (Optional) Torso control, as before
  try {
    // double lift_value = 0.2;  // Example lift value, adjust as needed
    // node->TorsoControl(lift_value);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in TorsoControl: %s ", e.what());
  }

  // Publisher for planning scene (if needed)
  auto planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", rclcpp::QoS(1));

  // Add obstacle (if needed)
  geometry_msgs::msg::PoseStamped obstacle_pose;
  obstacle_pose.header.frame_id = "base_footprint";
  obstacle_pose.pose.position.x = 0.4;
  obstacle_pose.pose.position.y = 0.0;
  obstacle_pose.pose.position.z = 0.5;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
    node->Add_Obstacle(obstacle_pose, "Table");
    
  planning_scene_publisher_->publish(planning_scene_msg);

  // Create a subscription to the target pose topic using the dedicated callback
  auto pose_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "/target_pose", 10,
    [node](const geometry_msgs::msg::Pose::SharedPtr msg) {
      target_pose_callback(msg, node);
    });

    // Create a subscription to the gripper command topic
  auto gripper_cmd_sub = node->create_subscription<std_msgs::msg::String>(
    "/gripper_cmd", 10,
    [node](const std_msgs::msg::String::SharedPtr msg) {
      gripper_cmd_callback(msg, node);
  });

  // Spin to process callbacks
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
// ...existing code...
