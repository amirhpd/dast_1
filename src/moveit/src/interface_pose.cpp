// Node to send command to moveit.
// Moves the end effector to the given pose.
// Humble only supports the moveit interface with C++
// commands to run:
// ros2 launch description gazebo.launch.py 
// ros2 launch moveit moveit.launch.py 
// ros2 run moveit interface_pose 2.094 -1.345 0.038 -3.053 0.000 1.000
// valid pose examples:
// 0.0 0.0 7.23 0.0 0.0 0.0
// 2.094 -1.345 0.038 -3.053 0.000 1.000
// 3.112 2.400 4.501 -0.987 0.708 -0.508
// -4.433 0.635 1.326 -0.000 -1.130 -0.142


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

void move_robot_to_pose(
    const std::shared_ptr<rclcpp::Node> node, 
    float x, float y, float z, float roll, float pitch, float yaw)
{

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("rclcpp");
    std::shared_ptr<rclcpp::Executor> executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options("manipulator", "robot_description", "/");
    auto manipulator_move_group = moveit::planning_interface::MoveGroupInterface(node, mgi_options);
    manipulator_move_group.setPlannerId("RRTConnect");  // default
    manipulator_move_group.setPlanningTime(30.0);
    manipulator_move_group.setMaxVelocityScalingFactor(0.1);
    manipulator_move_group.setMaxAccelerationScalingFactor(0.1);
    manipulator_move_group.setNumPlanningAttempts(10); 
    manipulator_move_group.setGoalPositionTolerance(0.001); 

    const moveit::core::JointModelGroup* joint_model_group =
      manipulator_move_group.getCurrentState()->getJointModelGroup("manipulator");


    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools(node, "world", "display_planned_path", manipulator_move_group.getRobotModel());
    // visual_tools.deleteAllMarkers();
    // visual_tools.loadRemoteControl();
    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "MoveGroupInterface Tutorial", rvt::WHITE, rvt::XLARGE);
    // visual_tools.trigger();

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(quaternion);
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = quaternion_msg;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    bool manipulator_at_goal = manipulator_move_group.setPoseTarget(target_pose, "tip");

    if (!manipulator_at_goal)
    {
        RCLCPP_WARN(rclcpp::get_logger("interface_pose"), "INVALID GOAL!");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
    moveit::core::MoveItErrorCode plan_result = manipulator_move_group.plan(manipulator_plan);

    /* Visualize the plan in RViz */
    auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 100);

    // visual_tools.publishAxisLabeled(target_pose, "pose");
    // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(manipulator_plan.trajectory_, joint_model_group);
    // visual_tools.publishPath(manipulator_plan.trajectory_, rvt::LIME_GREEN, rvt::SMALL);
    // for (std::size_t i = 0; i < manipulator_plan.trajectory_.joint_trajectory.points.size(); ++i)
    // {
    //     // Visualize the target pose as a green sphere of size 0.03m
    //     visual_tools.publishSphere(manipulator_plan.trajectory_.joint_trajectory.points[i].positions, 0.03, rvt::GREEN);
    // }
    // visual_tools.trigger();

    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {

        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        display_trajectory.model_id = "dast_1"; 
        display_trajectory.trajectory_start = manipulator_plan.start_state_;
        display_trajectory.trajectory.push_back(manipulator_plan.trajectory_);
        display_publisher->publish(display_trajectory);
        RCLCPP_INFO(rclcpp::get_logger("interface_pose"), "TRAJECTORY PUBLISHED!");

        RCLCPP_INFO(rclcpp::get_logger("interface_pose"), "MANIPULATOR PLAN SUCCEEDED!");
        RCLCPP_INFO(rclcpp::get_logger("interface_pose"), moveit::core::error_code_to_string(plan_result).c_str());
        // manipulator_move_group.move();
        manipulator_move_group.execute(manipulator_plan);  // both move() and execute() work
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("interface_pose"), "MANIPULATOR PLAN FAILED!");
        RCLCPP_ERROR(rclcpp::get_logger("interface_pose"), moveit::core::error_code_to_string(plan_result).c_str());
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    auto node = rclcpp::Node::make_shared("interface_pose", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    if (argc != 7)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: <x> <y> <z> <roll> <pitch> <yaw>");
        return 1;
    }

    float x = std::stof(argv[1]);
    float y = std::stof(argv[2]);
    float z = std::stof(argv[3]);
    float roll = std::stof(argv[4]);
    float pitch = std::stof(argv[5]);
    float yaw = std::stof(argv[6]);

    move_robot_to_pose(node, x, y, z, roll, pitch, yaw);

    rclcpp::shutdown();
    return 0;
}


// still does not show the planned path in rviz, but the topic /display_planned_path gets values and published.
// to re-produce:
// ros2 launch description gazebo.launch.py
// ros2 launch moveit moveit.launch.py
// ros2 run moveit interface_pose 2.094 -1.345 0.038 -3.053 0.000 1.000
// ros2 topic echo /display_planned_path
