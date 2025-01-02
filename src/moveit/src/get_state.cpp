// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    auto node = rclcpp::Node::make_shared("get_state", node_options);
    const auto& LOGGER = node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();


    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    // const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");


    auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options("manipulator", "robot_description", "/");
    auto manipulator_move_group = moveit::planning_interface::MoveGroupInterface(node, mgi_options);

    moveit::core::RobotStatePtr current_state = manipulator_move_group.getCurrentState(10);

    // const moveit::core::JointModelGroup* joint_model_group =
    //   manipulator_move_group.getCurrentState()->getJointModelGroup("manipulator");

    rclcpp::shutdown();
    return 0;

}