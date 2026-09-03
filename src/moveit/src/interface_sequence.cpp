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
#include <moveit/utils/moveit_error_code.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <sstream>
#include <string>
#include <vector>

using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
const std::string pipeline_id = "pilz_industrial_motion_planner";
const std::string action_name = "sequence_move_group";

// Returned by send_sequence() when the goal never reached the planner at all,
// so it cannot be confused with any real MoveItErrorCodes value.
const int TRANSPORT_FAILURE = -100000;


// "segment 2 (PTP, pose [1.992, -5.473, ...])" -- used in the error messages so
// the log points at a line of the YAML file instead of at the sequence as a whole.
std::string describe_segment(const YAML::Node &segment, size_t index)
{
    std::ostringstream text;
    text << "segment " << index << " (" << segment["type"].as<std::string>("PTP");

    const char *key = segment["joints"] ? "joints" : (segment["pose"] ? "pose" : nullptr);
    if (key)
    {
        text << ", " << key << " [";
        std::vector<double> values = segment[key].as<std::vector<double>>();
        for (size_t i = 0; i < values.size(); i++)
        {
            text << (i ? ", " : "") << values[i];
        }
        text << "]";
    }
    text << ")";
    return text.str();
}


// Puts the segment's target -- joint values or a pose -- onto the move group.
void apply_target(MoveGroupInterface &move_group, const YAML::Node &segment)
{
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
}


// Turns one YAML segment into a MotionSequenceItem, using MoveGroupInterface to
// fill in the boilerplate of the plan request.
moveit_msgs::msg::MotionSequenceItem build_item(
    MoveGroupInterface &move_group,
    const YAML::Node &segment,
    bool is_first)
{
    const std::string type = segment["type"].as<std::string>("PTP");
    move_group.setPlannerId(type);
    apply_target(move_group, segment);

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


// Builds the goal for the first 'count' segments. Pilz rejects the whole
// sequence unless the final blend radius is zero, so it is always forced there.
MoveGroupSequence::Goal build_goal(
    MoveGroupInterface &move_group,
    const YAML::Node &segments,
    size_t count,
    bool plan_only,
    const rclcpp::Logger &logger)
{
    MoveGroupSequence::Goal goal;
    for (size_t i = 0; i < count; i++)
    {
        goal.request.items.push_back(build_item(move_group, segments[i], i == 0));
    }

    double &last_blend = goal.request.items.back().blend_radius;
    if (last_blend != 0.0)
    {
        RCLCPP_WARN(logger, "Last segment had blend_radius %.3f; forcing it to 0.", last_blend);
        last_blend = 0.0;
    }

    goal.planning_options.plan_only = plan_only;
    return goal;
}


// Sends one goal and waits for it. Returns the MoveItErrorCodes value, or
// TRANSPORT_FAILURE if the goal never got as far as the planner.
int send_sequence(
    const std::shared_ptr<rclcpp::Node> node,
    rclcpp_action::Client<MoveGroupSequence>::SharedPtr client,
    const MoveGroupSequence::Goal &goal)
{
    auto goal_handle_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send the goal.");
        return TRANSPORT_FAILURE;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by the server.");
        return TRANSPORT_FAILURE;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the result.");
        return TRANSPORT_FAILURE;
    }

    return result_future.get().result->response.error_code.val;
}


// Checks a 'joints' segment against the robot model. MoveGroupInterface silently
// clamps out-of-range joint values, so a plan of the clamped target succeeds
// while Pilz still rejects the original request -- this catches that case.
// Returns an empty string when the target is fine.
std::string check_joints(MoveGroupInterface &move_group, const YAML::Node &segment)
{
    std::vector<double> joints = segment["joints"].as<std::vector<double>>();
    const moveit::core::JointModelGroup *group =
        move_group.getRobotModel()->getJointModelGroup(move_group.getName());
    const std::vector<const moveit::core::JointModel *> &models = group->getActiveJointModels();

    std::ostringstream problem;
    if (joints.size() != models.size())
    {
        problem << "'joints' has " << joints.size() << " values but group '" << group->getName()
                << "' has " << models.size() << " joints.";
        return problem.str();
    }

    for (size_t i = 0; i < joints.size(); i++)
    {
        const moveit::core::VariableBounds &bounds = models[i]->getVariableBounds()[0];
        if (joints[i] < bounds.min_position_ || joints[i] > bounds.max_position_)
        {
            problem << models[i]->getName() << " = " << joints[i] << " is outside its limits ["
                    << bounds.min_position_ << ", " << bounds.max_position_ << "].";
            return problem.str();
        }
    }
    return "";
}


// Asks move_group's /compute_ik service whether the segment's pose has any
// joint solution at all. Returns a MoveItErrorCodes value.
int check_ik(
    const std::shared_ptr<rclcpp::Node> node,
    MoveGroupInterface &move_group,
    const YAML::Node &segment)
{
    auto client = node->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service 'compute_ik' not available.");
        return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }

    std::vector<double> p = segment["pose"].as<std::vector<double>>();
    tf2::Quaternion quaternion;
    quaternion.setRPY(p[3], p[4], p[5]);

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = move_group.getName();
    request->ik_request.ik_link_name = move_group.getEndEffectorLink();
    request->ik_request.robot_state.is_diff = true;
    request->ik_request.avoid_collisions = true;
    request->ik_request.timeout = rclcpp::Duration::from_seconds(1.0);
    request->ik_request.pose_stamped.header.frame_id = move_group.getPlanningFrame();
    request->ik_request.pose_stamped.pose.orientation = tf2::toMsg(quaternion);
    request->ik_request.pose_stamped.pose.position.x = p[0];
    request->ik_request.pose_stamped.pose.position.y = p[1];
    request->ik_request.pose_stamped.pose.position.z = p[2];

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Call to 'compute_ik' failed.");
        return moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    }
    return future.get()->error_code.val;
}


// The sequence action reports every planner problem as the same opaque FAILURE
// (99999), because Pilz turns the planner's real error code into an exception
// before move_group sees it. This re-plans the sequence one segment longer each
// time to find the segment that breaks it, then plans that segment on its own --
// a plain plan request does return the real error code (NO_IK_SOLUTION,
// GOAL_IN_COLLISION, ...).
void diagnose_failure(
    const std::shared_ptr<rclcpp::Node> node,
    MoveGroupInterface &move_group,
    rclcpp_action::Client<MoveGroupSequence>::SharedPtr client,
    const YAML::Node &segments)
{
    const rclcpp::Logger logger = node->get_logger();
    RCLCPP_INFO(logger, "Looking for the segment that failed..");

    size_t failing = 0;
    bool found = false;
    for (size_t count = 1; count <= segments.size(); count++)
    {
        MoveGroupSequence::Goal goal = build_goal(move_group, segments, count, true, logger);
        if (send_sequence(node, client, goal) != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            failing = count - 1;
            found = true;
            break;
        }
    }

    if (!found)
    {
        RCLCPP_ERROR(logger,
                     "Every segment plans on its own, so the sequence failed while blending or "
                     "executing. If any blend_radius is non-zero, set it to 0: blending needs an "
                     "IK solution for each sampled corner pose, which this 5-DOF arm rarely has.");
        return;
    }

    RCLCPP_ERROR(logger, "The sequence first fails at %s.",
                 describe_segment(segments[failing], failing).c_str());

    // Ask move_group for the real reason. A pose target is almost always an IK
    // problem, and /compute_ik answers that directly; a joint target can only
    // fail on limits or collision, which a plain OMPL plan does report.
    int code = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    if (segments[failing]["pose"])
    {
        code = check_ik(node, move_group, segments[failing]);
        RCLCPP_ERROR(logger, "Inverse kinematics for that pose reports: %s (%d)",
                     moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(code)).c_str(),
                     code);
    }
    else
    {
        const std::string problem = check_joints(move_group, segments[failing]);
        if (!problem.empty())
        {
            RCLCPP_ERROR(logger, "%s", problem.c_str());
            return;
        }

        move_group.setPlanningPipelineId("ompl");
        move_group.setPlannerId("RRTConnect");
        apply_target(move_group, segments[failing]);

        MoveGroupInterface::Plan plan;
        code = move_group.plan(plan).val;
        RCLCPP_ERROR(logger, "Planning that segment alone reports: %s (%d)",
                     moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(code)).c_str(),
                     code);
    }

    switch (code)
    {
        case moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION:
        case moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION:
            RCLCPP_ERROR(logger, "The waypoint collides with an object in the planning scene.");
            break;
        case moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        case moveit_msgs::msg::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
            RCLCPP_ERROR(logger,
                         "The target is outside the joint limits (+/-pi/2 on every joint) or the "
                         "'joints' list does not have 5 entries.");
            break;
    }
}


bool run_sequence(const std::shared_ptr<rclcpp::Node> node, const std::string &yaml_path)
{
    const rclcpp::Logger logger = node->get_logger();
    YAML::Node config = YAML::LoadFile(yaml_path);
    const std::string group = config["group"].as<std::string>("manipulator");

    auto move_group = MoveGroupInterface(node, group);
    move_group.setMaxVelocityScalingFactor(config["velocity_scaling"].as<double>(0.1));
    move_group.setMaxAccelerationScalingFactor(config["acceleration_scaling"].as<double>(0.1));

    const YAML::Node segments = config["segments"];
    if (!segments || segments.size() == 0)
    {
        RCLCPP_ERROR(logger, "No 'segments' found in %s", yaml_path.c_str());
        return false;
    }

    auto client = rclcpp_action::create_client<MoveGroupSequence>(node, action_name);
    if (!client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(logger, "Action server '%s' not available.", action_name.c_str());
        return false;
    }

    MoveGroupSequence::Goal goal = build_goal(move_group, segments, segments.size(), false, logger);
    RCLCPP_INFO(logger, "Sending sequence of %zu segments..", goal.request.items.size());

    const int error_code = send_sequence(node, client, goal);
    if (error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_INFO(logger, "SEQUENCE SUCCEEDED!");
        return true;
    }
    if (error_code == TRANSPORT_FAILURE)
    {
        return false;
    }

    RCLCPP_ERROR(logger, "SEQUENCE FAILED! %s (%d)",
                 moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(error_code)).c_str(),
                 error_code);
    diagnose_failure(node, move_group, client, segments);
    return false;
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
