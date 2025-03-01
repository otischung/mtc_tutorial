#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif
#if __has_include(<moveit/planning_scene_interface/planning_scene_interface.hpp>)
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#else
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

    // // Set a example target Pose
    // auto const target_pose = [] {
    //     geometry_msgs::msg::Pose msg;
    //     // Set orientation to identity (no rotation)
    //     msg.orientation.x = 0.0;
    //     msg.orientation.y = 0.0;
    //     msg.orientation.z = 0.0;
    //     msg.orientation.w = 1.0;
    //     // Set position values
    //     msg.position.x = 0.2614998195467109;
    //     msg.position.y = -0.26450008964248334;
    //     msg.position.z = 1.0426919117050755;

    //     // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d using tf2::fromMsg
    //     Eigen::Isometry3d eigen_transform;
    //     tf2::fromMsg(msg, eigen_transform);

    //     // Define a 90-degree (π/2 radians) rotation about the Z-axis
    //     double angle_rad = M_PI / 2.0;  // 90 degrees in radians
    //     Eigen::AngleAxisd rotation_vector(angle_rad, Eigen::Vector3d::UnitZ());

    //     // Apply the rotation to the original transform
    //     eigen_transform.rotate(rotation_vector);

    //     // Convert the Eigen transform back to geometry_msgs::msg::Pose
    //     geometry_msgs::msg::Pose rotated_pose = tf2::toMsg(eigen_transform);
    //     return rotated_pose;
    // }();
    // move_group_interface.setPoseTarget(target_pose);

    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id =
                                       move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.284;
        primitive.dimensions[primitive.BOX_Y] = 1.3;
        primitive.dimensions[primitive.BOX_Z] = 1.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.36 + primitive.dimensions[primitive.BOX_X] / 2.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.0 + primitive.dimensions[primitive.BOX_Z] / 2.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    auto const init_pose = [] {
        // This is the kinematic result of the joint angles
        // [28, 88, -45, -80, -45, -82] in degrees.
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3178071837760669;
        msg.position.y = -0.10556869093811046;
        msg.position.z = 1.1215113837188624;
        msg.orientation.x = -0.1345047736672149;
        msg.orientation.y = -0.1247301629367219;
        msg.orientation.z = 0.6927253372844873;
        msg.orientation.w = 0.697482945596954;
        return msg;
    }();
    move_group_interface.setPoseTarget(init_pose);

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // planning_scene_interface.applyCollisionObject(collision_object);  // Updates synchronously.
    planning_scene_interface.addCollisionObjects({collision_object});  // Updates asynchronously.

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

    // Define waypoints for the Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(init_pose);  // starting pose

    // Create a new pose that is a translation along the desired axis (e.g., y-axis)
    auto target_pose = init_pose;
    target_pose.position.y -= 0.4;
    waypoints.push_back(target_pose);  // end pose

    // Compute the Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;  // distance between interpolated points
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);

    if (fraction == 1.0) {
        move_group_interface.execute(trajectory);
    } else {
        RCLCPP_ERROR(logger, "Cartesian path planning failed to compute the complete path.");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
