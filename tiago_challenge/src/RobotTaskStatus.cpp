#include "RobotTaskStatus.hpp"

// Default constructor
RobotTaskStatus::RobotTaskStatus()
: status_(Status::QUEUED), arm_(Arm::ARM)
{
  // goal_ is default-initialized
}

// Parameterized constructor
RobotTaskStatus::RobotTaskStatus(const geometry_msgs::msg::Pose & goal, Status status, Arm arm)
: goal_(goal), status_(status), arm_(arm) {}

// Setters
void RobotTaskStatus::setGoal(const geometry_msgs::msg::Pose & goal)
{
  goal_ = goal;
}

void RobotTaskStatus::setStatus(Status status)
{
  status_ = status;
}

void RobotTaskStatus::setArm(Arm arm)
{
  arm_ = arm;
}

// Getters
geometry_msgs::msg::Pose RobotTaskStatus::getGoal() const
{
  return goal_;
}

RobotTaskStatus::Status RobotTaskStatus::getStatus() const
{
  return status_;
}

RobotTaskStatus::Arm RobotTaskStatus::getArm() const
{
  return arm_;
}

// Optional string conversion functions
std::string RobotTaskStatus::statusToString(Status status)
{
  switch (status) {
    case Status::QUEUED:        return "queued";
    case Status::WAITING_BRICK: return "waiting_brick";
    case Status::GRASPED_BRICK: return "grasped_brick";
    case Status::ONGOING:       return "ongoing";
    case Status::LEAVE_BRICK:   return "leave_brick";
    case Status::DONE:          return "done";
    default:                    return "unknown";
  }
}

std::string RobotTaskStatus::armToString(Arm arm)
{
  switch (arm) {
    case Arm::ARM:        return "arm";
    case Arm::ARM_torso: return "arm_torso";
    default:                   return "unknown";
  }
}
