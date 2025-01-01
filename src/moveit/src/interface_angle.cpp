// Node to send command to moveit.
// Moves the joint by the given angles.
// Humble only supports the moveit interface with C++
// commands to run:
// ros2 launch description gazebo.launch.py 
// ros2 launch moveit moveit.launch.py 
// ros2 run moveit interface_angle 1.0 0.52 1.57 1.57 0.0


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


void move_robot_by_angle(
    const std::shared_ptr<rclcpp::Node> node, 
    float joint_1, float joint_2, float joint_3, float joint_4, float joint_5
    )
{
    auto manipulator_move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");

    std::vector<double> manipulator_goal {joint_1, joint_2, joint_3, joint_4, joint_5};
    bool manipulator_at_goal = manipulator_move_group.setJointValueTarget(manipulator_goal);

    if (!manipulator_at_goal)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "INVALID GOAL!");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
    moveit::core::MoveItErrorCode plan_result = manipulator_move_group.plan(manipulator_plan);

    if(plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MANIPULATOR PLAN SUCCEEDED!");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), moveit::core::error_code_to_string(plan_result).c_str());
        manipulator_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "MANIPULATOR PLAN FAILED!");
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), moveit::core::error_code_to_string(plan_result).c_str());
        return;
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

    if (argc != 6)
        {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: <joint_1> <joint_2> <joint_3> <joint_4> <joint_5>");
        return 1;
        }
    float joint_1 = std::stof(argv[1]);
    float joint_2 = std::stof(argv[2]);
    float joint_3 = std::stof(argv[3]);
    float joint_4 = std::stof(argv[4]);
    float joint_5 = std::stof(argv[5]);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("interface_angle");
    move_robot_by_angle(node, joint_1, joint_2, joint_3, joint_4, joint_5);

    rclcpp::shutdown();
}
