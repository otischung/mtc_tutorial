#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <Eigen/Geometry>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "right_arm");

    // Set a target Pose
    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        // Set orientation to identity (no rotation)
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        // Set position values
        msg.position.x = 0.3;
        msg.position.y = -0.3;
        msg.position.z = 1.0;

        // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d using tf2::fromMsg
        Eigen::Isometry3d eigen_transform;
        tf2::fromMsg(msg, eigen_transform);

        // Define a 90-degree (π/2 radians) rotation about the Z-axis
        double angle_rad = M_PI / 2.0;  // 90 degrees in radians
        Eigen::AngleAxisd rotation_vector(angle_rad, Eigen::Vector3d::UnitZ());

        // Apply the rotation to the original transform
        eigen_transform.rotate(rotation_vector);

        // Convert the Eigen transform back to geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose rotated_pose = tf2::toMsg(eigen_transform);
        return rotated_pose;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
