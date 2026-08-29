// Node to send a multi-segment trajectory to moveit.
// Plans every segment with collision checking (Pilz industrial motion planner),
// blends them into one continuous motion, and executes it.
// commands to run:
// ros2 launch description gazebo.launch.py
// ros2 launch moveit moveit.launch.py
// ros2 run moveit interface_sequence <waypoints.yaml>
// see config/sequence_example.yaml for the file format.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
const std::string pipeline_id = "pilz_industrial_motion_planner";
const std::string action_name = "sequence_move_group";


// Turns one YAML segment into a MotionSequenceItem, using MoveGroupInterface to
// fill in the boilerplate of the plan request.
moveit_msgs::msg::MotionSequenceItem build_item(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const YAML::Node &segment,
    bool is_first)
{
    const std::string type = segment["type"].as<std::string>("PTP");
    move_group.setPlannerId(type);

    if (segment["joints"])
    {
        std::vector<double> joints = segment["joints"].as<std::vector<double>>();
        move_group.setJointValueTarget(joints);
    }
    else if (segment["pose"])
    {
        std::vector<double> p = segment["pose"].as<std::vector<double>>();
        if (p.size() != 6)
        {
            throw std::runtime_error("'pose' needs 6 values: x y z roll pitch yaw");
        }
        tf2::Quaternion quaternion;
        quaternion.setRPY(p[3], p[4], p[5]);
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation = tf2::toMsg(quaternion);
        target_pose.position.x = p[0];
        target_pose.position.y = p[1];
        target_pose.position.z = p[2];
        move_group.setPoseTarget(target_pose);
    }
    else
    {
        throw std::runtime_error("each segment needs either 'joints' or 'pose'");
    }

    moveit_msgs::msg::MotionSequenceItem item;
    move_group.constructMotionPlanRequest(item.req);
    item.req.pipeline_id = pipeline_id;
    item.req.planner_id = type;
    item.blend_radius = segment["blend_radius"].as<double>(0.0);

    // Pilz takes the start state from the first item only; the rest continue
    // from wherever the previous segment ended.
    if (!is_first)
    {
        item.req.start_state = moveit_msgs::msg::RobotState();
        item.req.start_state.is_diff = true;
    }

    return item;
}


bool run_sequence(const std::shared_ptr<rclcpp::Node> node, const std::string &yaml_path)
{
    YAML::Node config = YAML::LoadFile(yaml_path);
    const std::string group = config["group"].as<std::string>("manipulator");

    auto move_group = moveit::planning_interface::MoveGroupInterface(node, group);
    move_group.setMaxVelocityScalingFactor(config["velocity_scaling"].as<double>(0.1));
    move_group.setMaxAccelerationScalingFactor(config["acceleration_scaling"].as<double>(0.1));

    const YAML::Node segments = config["segments"];
    if (!segments || segments.size() == 0)
    {
        RCLCPP_ERROR(node->get_logger(), "No 'segments' found in %s", yaml_path.c_str());
        return false;
    }

    MoveGroupSequence::Goal goal;
    for (size_t i = 0; i < segments.size(); i++)
    {
        goal.request.items.push_back(build_item(move_group, segments[i], i == 0));
    }

    // Pilz rejects the whole sequence unless the final blend radius is zero.
    double &last_blend = goal.request.items.back().blend_radius;
    if (last_blend != 0.0)
    {
        RCLCPP_WARN(node->get_logger(),
                    "Last segment had blend_radius %.3f; forcing it to 0.", last_blend);
        last_blend = 0.0;
    }

    goal.planning_options.plan_only = false;

    auto client = rclcpp_action::create_client<MoveGroupSequence>(node, action_name);
    if (!client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Action server '%s' not available.", action_name.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Sending sequence of %zu segments..", goal.request.items.size());
    auto goal_handle_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send the goal.");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by the server.");
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the result.");
        return false;
    }

    auto result = result_future.get();
    const int error_code = result.result->response.error_code.val;
    if (error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "SEQUENCE FAILED! MoveItErrorCode: %d", error_code);
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "SEQUENCE SUCCEEDED!");
    return true;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: <waypoints.yaml>");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("interface_sequence");

    bool success = false;
    try
    {
        success = run_sequence(node, argv[1]);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}
