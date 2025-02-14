// Node to create an action. Runs tasks according to the received goal number.
// To run use task_interface launch file.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "msgs/action/task_action.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


using namespace std::placeholders;

namespace task
{
class TaskServerAngle : public rclcpp::Node
{
public:
  explicit TaskServerAngle(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server_angle", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<msgs::action::TaskAction>(
        this, "task_server_angle", 
        std::bind(&TaskServerAngle::goalCallback, this, _1, _2),
        std::bind(&TaskServerAngle::cancelCallback, this, _1),
        std::bind(&TaskServerAngle::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<msgs::action::TaskAction>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> manipulator_move_group_;
  std::vector<double> manipulator_joint_goal_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const msgs::action::TaskAction::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::TaskAction>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(manipulator_move_group_){
      manipulator_move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::TaskAction>> goal_handle)
  {
    std::thread{ std::bind(&TaskServerAngle::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::TaskAction>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<msgs::action::TaskAction::Result>();

    // MoveIt 2 Interface
    if(!manipulator_move_group_){
      manipulator_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");
    }
  
    if (goal_handle->get_goal()->task_number == 0)
    {
      manipulator_joint_goal_ = {0.0, 0.785, 0.716, 1.57, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 1)
    {
      manipulator_joint_goal_ = {0.872, 0.244, 0.488, 1.169, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 2)
    {
      manipulator_joint_goal_ = {0.872, 1.151, 0.087, 1.57, 0.0};
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      return;
    }

    manipulator_move_group_->setStartState(*manipulator_move_group_->getCurrentState());

    bool manipulator_within_bounds = manipulator_move_group_->setJointValueTarget(manipulator_joint_goal_);
    if (!manipulator_within_bounds)
    {
      RCLCPP_WARN(get_logger(),
                  "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
    bool manipulator_plan_success = (manipulator_move_group_->plan(manipulator_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(manipulator_plan_success)
    {
      RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the manipulatore and the gripper");
      manipulator_move_group_->move();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "One or more planners failed!");
      return;
    }
  
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
};
}  // namespace task

RCLCPP_COMPONENTS_REGISTER_NODE(task::TaskServerAngle)