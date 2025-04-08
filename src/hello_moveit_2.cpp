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

int main(int argc, char *argv[]) {
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
    planning_scene_interface.applyCollisionObject(collision_object);

    // --------------------------------------------------
    // Cartesian planning from current pose to init_pose
    // --------------------------------------------------
    // Get the current pose of the end effector
    auto current_pose_stamped = move_group_interface.getCurrentPose();
    auto current_pose = current_pose_stamped.pose;

    // Define waypoints from the current pose to the initial pose
    std::vector<geometry_msgs::msg::Pose> waypoints_to_init;
    waypoints_to_init.push_back(current_pose);
    waypoints_to_init.push_back(init_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;  // step size for interpolating along the Cartesian path
#ifdef ROS_DISTRO_JAZZY
    // Prevent deprecated warning in Jazzy
    double fraction = move_group_interface.computeCartesianPath(waypoints_to_init, eef_step, trajectory);
#else
    double fraction = move_group_interface.computeCartesianPath(waypoints_to_init, eef_step, 0.0, trajectory);
#endif
    if (fraction == 1.0) {
        RCLCPP_INFO(logger, "Cartesian path from current pose to init_pose computed successfully.");
        move_group_interface.execute(trajectory);
    } else {
        RCLCPP_ERROR(logger, "Failed to compute complete Cartesian path from current pose to init_pose.");
    }

    // --------------------------------------------------
    // Cartesian planning from init_pose to target_pose
    // --------------------------------------------------
    // Define the target pose by modifying the init_pose (e.g., translating along the y-axis)
    auto target_pose = init_pose;
    target_pose.position.y -= 0.4;

    std::vector<geometry_msgs::msg::Pose> waypoints_to_target;
    waypoints_to_target.push_back(init_pose);
    waypoints_to_target.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory2;
#ifdef ROS_DISTRO_JAZZY
    // Prevent deprecated warning in Jazzy
    fraction = move_group_interface.computeCartesianPath(waypoints_to_target, eef_step, trajectory2);
#else
    fraction = move_group_interface.computeCartesianPath(waypoints_to_target, eef_step, 0.0, trajectory2);
#endif
    if (fraction == 1.0) {
        RCLCPP_INFO(logger, "Cartesian path from init_pose to target_pose computed successfully.");
        move_group_interface.execute(trajectory2);
    } else {
        RCLCPP_ERROR(logger, "Failed to compute complete Cartesian path from init_pose to target_pose.");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
