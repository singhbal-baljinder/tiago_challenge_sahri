/* Including General C++ Header(s) */
#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>

/* Including ROS Header(s) */
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "std_msgs/msg/string.hpp"


#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

#include "RobotTaskStatus.hpp"


// // for planning scene
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/GetStateValidity.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/ApplyPlanningScene.h>


/* include Icarso Header */

// Our Action interface type for moving TIAGo++'s Gripper, provided as a typedef for convenience
using PointGripperClient = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>;
using PointGripperClientPtr = std::shared_ptr<PointGripperClient>;

// Our Action interface type for moving TIAGo++'s Gripper, provided as a typedef for convenience
using gripper_control_client = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>;
using gripper_control_client_ptr = std::shared_ptr<gripper_control_client>;

// Our Action interface type for moving TIAGo++'s Gripper, provided as a typedef for convenience
using torso_control_client = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>;
using torso_control_client_ptr = std::shared_ptr<torso_control_client>;


#define ARM_LEFT  0x00
#define ARM_RIGHT 0x01
#define BOTH_ARMS 0x02

class icr_Motionplanning_arms : public rclcpp::Node
{
public:
  icr_Motionplanning_arms()
  : Node("icr_motionplanning_arms")
  {
  }

  ~icr_Motionplanning_arms() override = default;


  /* ----------- Gripper Position Control ---------- */
  void GripperControl(const std::string & command);
  void createGripperClient(
    gripper_control_client_ptr & client,
    const std::string & controller_name);
  void SendGripperAction(
    gripper_control_client_ptr client,
    const std::vector<std::string> & joint_names,
    const std::string & command);

  /* ----------- Torso Control ---------- */
  void TorsoMotionPlanning(void);
  void TorsoControl(double lift_value);
  void TorsoSendAction(torso_control_client_ptr torso_client, double lift_value);

  /* ----------- ARM Position Control ---------- */
  void  ARM_Homeposition(void);
  int   ArmMotionPlanning(RobotTaskStatus);

  int   Box_Pose_Verify(void);
  void  Pose_X_Arm(int motion_type);
  void  Pose_Y_Arm(int motion_type);
  void  Pose_Z_Arm(int motion_type);
  void  Stable_Pose(int motion_type);


  bool motion_planning_control(
    const geometry_msgs::msg::Pose & goal,
    RobotTaskStatus::Arm motion_type);

  moveit_msgs::msg::PlanningScene Add_Obstacle(
    const geometry_msgs::msg::PoseStamped & obstacle_pose,
    const std::string & obstacle_id);

  void  motion_planning_control_fast(int motion_type);
  // void Set_initial_motion(int motion_type);
  // void Approaching_to_initial_pose(int motion_type);
  void Set_initial_motion();
  void Approaching_to_initial_pose();
  void Goal_Arms_Absolute(
    int caller, double Target_Left_Arm_pose[3],
    double Target_Right_Arm_pose[3], double Target_Left_Orientation[3],
    double Target_Right_Orientation[3]);
  // void icr_Motionplanning_arms::Goal_Arms_Absolute_quaternion(double Target_Left_Arm_pose[3], double Target_Right_Arm_pose[3],tf::Quaternion quat_left,tf::Quaternion quat_right);


};
