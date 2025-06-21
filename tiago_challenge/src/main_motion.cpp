#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

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

// Class to encapsulate callbacks and subscriptions
class MotionCallbacks
{
public:
  MotionCallbacks(std::shared_ptr<icr_Motionplanning_arms> node, rclcpp::Node::SharedPtr base_node)
  : node_(node)
  {
    // Subscription for target pose
    pose_sub_ = base_node->create_subscription<geometry_msgs::msg::Pose>(
      "/target_pose", 10,
      std::bind(&MotionCallbacks::target_pose_callback, this, std::placeholders::_1));

    // Subscription for gripper command
    gripper_cmd_sub_ = base_node->create_subscription<std_msgs::msg::String>(
      "/gripper_cmd", 10,
      std::bind(&MotionCallbacks::gripper_cmd_callback, this, std::placeholders::_1));

    //Publisher for move finished status
    move_finished_pub = base_node->create_publisher<std_msgs::msg::Bool>(
    "/move_finished", 10);

    RCLCPP_INFO(node_->get_logger(), "MotionCallbacks initialized with subscriptions.");
  }

  void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Received target pose from topic.");
    try {
      move_finished_ = node_->motion_planning_control(*msg, RobotTaskStatus::Arm::ARM_torso);
      RCLCPP_INFO(node_->get_logger(), "Motion planning succeeded.");
      RCLCPP_INFO(node_->get_logger(), "Target pose: [x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f]",
          msg->position.x, msg->position.y, msg->position.z,
          msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    } catch (const std::exception & e) {
      move_finished_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Motion planning failed: %s", e.what());
    }
    // Publisher for planning scene (if needed)
    auto planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(1));

    double safety_distance = 0.01; // Example safety distance, adjust as needed
    // Add obstacle (if needed)
    geometry_msgs::msg::PoseStamped obstacle_pose;
    obstacle_pose.header.frame_id = "base_footprint";
    obstacle_pose.pose.position.x = 0.90;
    obstacle_pose.pose.position.y = 0.0;
    obstacle_pose.pose.position.z = msg->position.z + safety_distance;
    obstacle_pose.pose.orientation.x = 0.0;
    obstacle_pose.pose.orientation.y = 0.0;
    obstacle_pose.pose.orientation.z = 0.0;
    obstacle_pose.pose.orientation.w = 1.0;
    moveit_msgs::msg::PlanningScene planning_scene_msg =
      node_->Add_Obstacle(obstacle_pose, "Table");

    planning_scene_publisher_->publish(planning_scene_msg);

    // Publish move_finished status
    std_msgs::msg::Bool move_finished_msg;
    move_finished_msg.data = move_finished_;
    move_finished_pub->publish(move_finished_msg);
  }

  void gripper_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Received gripper command: '%s'", msg->data.c_str());
    try {
      node_->GripperControl(msg->data);
      RCLCPP_INFO(node_->get_logger(), "Gripper command executed.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper command failed: %s", e.what());
    }
  }

  std::shared_ptr<icr_Motionplanning_arms> node_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_finished_pub;
    // Save success of move cmd
  bool move_finished_ = false;

};

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


  // Use MotionCallbacks to handle subscriptions and callbacks
  auto motion_callbacks = std::make_shared<MotionCallbacks>(node, node);

  // Spin to process callbacks
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
