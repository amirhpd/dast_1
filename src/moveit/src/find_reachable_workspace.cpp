#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


std::vector<double> get_fw_kinematics(const std::shared_ptr<rclcpp::Node> &node)
{
    tf2_ros::Buffer tf_buffer(node->get_clock());
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform("world", "end_effector", tf2::TimePointZero);
        auto translation = transform.transform.translation;
        auto rotation = transform.transform.rotation;
        return {translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w};
    }
    catch(const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PointCloudGenerator"), "Could not get transform: %s", ex.what());
        return {};
    }
}

void store_to_point_cloud(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    const auto &trajectory = plan.trajectory_.joint_trajectory.points;
    for (const auto &point : trajectory) {
        for (size_t i = 0; i < point.positions.size(); ++i) {
            std::cout << point.positions[i] << " ";
        }
        std::cout << std::endl;
        
        std::cout << trajectory[-1].positions[0] << std::endl;

        pcl::PointXYZ pcl_point;
        pcl_point.x = point.positions[0];
        pcl_point.y = point.positions[1]; 
        pcl_point.z = point.positions[2];              

        cloud.points.push_back(pcl_point);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1; // Unordered cloud
    cloud.is_dense = true;

    if (pcl::io::savePCDFile("points.pcd", cloud) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("PointCloudGenerator"), "Failed to save PCD file");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudGenerator"), "Saved point cloud");
    }

}


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
        // store_to_point_cloud(manipulator_plan);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MANIPULATOR PLAN SUCCEEDED!");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), moveit::core::error_code_to_string(plan_result).c_str());
        auto move_result = manipulator_move_group.move();

        if (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) 
        {
            std::vector fw_pose = get_fw_kinematics(node);
            std::cout << fw_pose[0] << fw_pose[1] << fw_pose[2] << fw_pose[3] << fw_pose[4] << fw_pose[5] << std::endl;
        }
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
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_sim_time", true);

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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("interface_angle", node_options);
    move_robot_by_angle(node, joint_1, joint_2, joint_3, joint_4, joint_5);

    rclcpp::shutdown();
}



// notes:
// the idea of using fw kinematics and move_robot_by_angle to get the reachable workspace is not correct
// tf2 library does not calculate the forward kinematics, it only provides the transformation between two frames
// so to get several end effector poses when passing the angles to the joints, each time the end effector should reach to that point first.

// The other idea should be tried out: 
// pass several poses to moveit, move_robot_to_pose function and store the ones that can be planned in the point cloud