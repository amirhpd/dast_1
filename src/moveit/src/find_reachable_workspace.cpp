// Performs a brute force search on joint angles to find a workspace reachable by the manipulator.
// The workspace is stored in a PointCloud file.

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl/io/pcd_io.h>


const std::string filename = "points.pcd";
const std::string node_name = "find_reachable_workspace";
const float pi = 3.14159265359f;


void store_to_point_cloud(const geometry_msgs::msg::Pose &target_pose)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud) == -1) {
        RCLCPP_WARN(rclcpp::get_logger(node_name), "Couldn't read file %s. Creating a new file instead.\n", filename.c_str());
    }

    pcl::PointXYZ pcl_point;
    pcl_point.x = target_pose.position.x;
    pcl_point.y = target_pose.position.y; 
    pcl_point.z = target_pose.position.z;          

    cloud.points.push_back(pcl_point);

    cloud.width = cloud.points.size();
    cloud.height = 1; // Unordered cloud
    cloud.is_dense = true;

    if (pcl::io::savePCDFile(filename, cloud) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger(node_name), "Failed to save PCD file");
    } else {
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Point saved to PointCloud: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }

}

geometry_msgs::msg::Pose get_pose_from_trajectory(
                                                const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                                const moveit::core::RobotStatePtr &robot_state,
                                                const std::string &end_effector_link
                                                )
{
    const auto &trajectory_points = plan.trajectory_.joint_trajectory.points;
    if (trajectory_points.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger(node_name), "Trajectory is empty!");
        return geometry_msgs::msg::Pose();
    }
    const auto &last_point = trajectory_points.back();
    const std::vector<double> &joint_positions = last_point.positions;
    robot_state->setVariablePositions(joint_positions);

    geometry_msgs::msg::Pose end_effector_pose;
    const Eigen::Isometry3d &end_effector_state = robot_state->getGlobalLinkTransform(end_effector_link);

    end_effector_pose.position.x = end_effector_state.translation().x();
    end_effector_pose.position.y = end_effector_state.translation().y();
    end_effector_pose.position.z = end_effector_state.translation().z();

    Eigen::Quaterniond orientation(end_effector_state.rotation());
    end_effector_pose.orientation.x = orientation.x();
    end_effector_pose.orientation.y = orientation.y();
    end_effector_pose.orientation.z = orientation.z();
    end_effector_pose.orientation.w = orientation.w();

    return end_effector_pose;
}

void brute_force_workspace(const std::shared_ptr<rclcpp::Node> &node)
{
    auto manipulator_move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
    manipulator_move_group.setPlannerId("RRTConnect");  // default
    manipulator_move_group.setPlanningTime(5.0);

    float resolution = 0.62831853071;
    int counter = 0;
    for(float joint_1 = -pi/2; joint_1 < pi/2; joint_1 += resolution/4){
        for(float joint_2 = -pi/2; joint_2 < pi/2; joint_2 += resolution){
            for(float joint_3 = -pi/2; joint_3 < pi/2; joint_3 += resolution){
                for(float joint_4 = -pi/2; joint_4 < pi/2; joint_4 += resolution){
                    for(float joint_5 = -pi/2; joint_5 < pi/2; joint_5 += resolution){
                        std::vector<double> joint_set {joint_1, joint_2, joint_3, joint_4, joint_5};
                        bool manipulator_at_goal = manipulator_move_group.setJointValueTarget(joint_set);
                        if (!manipulator_at_goal)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger(node_name), "Plan %i failed.", counter);
                            counter++;
                            continue; 
                        }
                        moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
                        moveit::core::MoveItErrorCode plan_result = manipulator_move_group.plan(manipulator_plan);
                        if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
                        {
                            geometry_msgs::msg::Pose pose = get_pose_from_trajectory(
                                manipulator_plan, 
                                manipulator_move_group.getCurrentState(), 
                                manipulator_move_group.getEndEffectorLink()
                                );
                            store_to_point_cloud(pose);
                            // manipulator_move_group.move()
                            counter++;
                        }
                        else
                        {
                            RCLCPP_ERROR(rclcpp::get_logger(node_name), "Point no. %d stored.", counter);
                            counter++;
                            continue;
                            // RCLCPP_ERROR(rclcpp::get_logger(node_name), moveit::core::error_code_to_string(plan_result).c_str());
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("use_sim_time", true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(node_name, node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    brute_force_workspace(node);

    rclcpp::shutdown();
}
