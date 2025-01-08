#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

const std::string filename = "points.pcd";
const std::string node_name = "find_reachable_workspace";


void store_to_point_cloud(const geometry_msgs::msg::Pose &target_pose)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger(node_name), "Couldn't read file %s. Creating a new file instead.\n", filename.c_str());
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


void plan_pose(
    const std::shared_ptr<rclcpp::Node> node, 
    float x, float y, float z, float roll, float pitch, float yaw)
{
    auto manipulator_move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
    manipulator_move_group.setPlannerId("RRTConnect");  // default
    manipulator_move_group.setPlanningTime(30.0);

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(quaternion);
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = quaternion_msg;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    bool manipulator_at_goal = manipulator_move_group.setPoseTarget(target_pose);

    if (!manipulator_at_goal)
    {
        RCLCPP_WARN(rclcpp::get_logger(node_name), "INVALID GOAL!");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
    moveit::core::MoveItErrorCode plan_result = manipulator_move_group.plan(manipulator_plan);

    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        store_to_point_cloud(target_pose);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(node_name), "Plan failed, skipping the point: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
        // RCLCPP_ERROR(rclcpp::get_logger(node_name), moveit::core::error_code_to_string(plan_result).c_str());
        return;
    }
}

void brute_force_workspace(const std::shared_ptr<rclcpp::Node> node)
{
    auto manipulator_move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
    manipulator_move_group.setPlannerId("RRTConnect");  // default
    manipulator_move_group.setPlanningTime(5.0);

    int counter = 0;
    for(float roll = -3.14; roll < 3.14; roll += 1.256)
    {
        for (float pitch = -3.14; pitch < 3.14; pitch += 1.256)
        {
            for (float yaw = -3.14; yaw < 3.14; yaw += 1.256)
            {
                for (float z = 0.4; z < 7.2; z += 0.72)
                {
                    for (float x = -4.2; x < 4.2; x += 1.2) 
                    {
                        for (float y = -4.2; y < 4.2; y += 1.2) 
                        {
                            // 2.094 -1.345 0.038 -3.053 0.000 1.000
                            tf2::Quaternion quaternion;
                            quaternion.setRPY(roll, pitch, yaw);
                            geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(quaternion);
                            geometry_msgs::msg::Pose target_pose;
                            target_pose.orientation = quaternion_msg;
                            target_pose.position.x = x;
                            target_pose.position.y = y;
                            target_pose.position.z = z;

                            bool manipulator_at_goal = manipulator_move_group.setPoseTarget(target_pose);

                            if (!manipulator_at_goal)
                            {
                                RCLCPP_WARN(rclcpp::get_logger(node_name), "Invalid goal %i, skipping the point: x=%f, y=%f, z=%f, r=%f, p=%f, y=%f,", counter, x, y, z, roll, pitch, yaw);
                                counter++;
                                continue; 
                            }
                                moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
                                moveit::core::MoveItErrorCode plan_result = manipulator_move_group.plan(manipulator_plan);
                            if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
                            {
                                store_to_point_cloud(target_pose);
                            }
                            else
                            {
                                RCLCPP_ERROR(rclcpp::get_logger(node_name), "Plan %i failed, skipping the point: x=%f, y=%f, z=%f, r=%f, p=%f, y=%f,", counter, x, y, z, roll, pitch, yaw);
                                counter++;
                                continue;
                                // RCLCPP_ERROR(rclcpp::get_logger(node_name), moveit::core::error_code_to_string(plan_result).c_str());
                            }
                            counter++;
                            RCLCPP_INFO(rclcpp::get_logger(node_name), "Point no. %d stored.", counter);
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

    brute_force_workspace(node);

    rclcpp::shutdown();
}

// This approach takes very long. for each wrong pose, plan function should reach to timeout.
// Another approach to test:
// Brute force on joint angles, so all combinations reach to a valid pose. 
// Then dothe fw kinematics somehow, to get the tip's pose
// fw kinematics can probably be done fast without moveit,
// or use movit plan but get the tip's pose from the plan fn without moving the robot.
