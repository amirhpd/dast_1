#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


void store_to_point_cloud(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    const auto &trajectory = plan.trajectory_.joint_trajectory.points;
    for (const auto &point : trajectory) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: %f %f %f %f %f %f",
                    point.positions[0], point.positions[1], point.positions[2],
                    point.positions[3], point.positions[4], point.positions[5]);
        
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
        store_to_point_cloud(manipulator_plan);
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
