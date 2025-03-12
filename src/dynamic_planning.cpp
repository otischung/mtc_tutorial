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
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.36 + primitive.dimensions[primitive.BOX_X] / 2.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 1.09 + primitive.dimensions[primitive.BOX_Z] / 2.0;

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

    // Scale to 50% of the maximum joint speeds and accelerations
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.5);

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

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // planning_scene_interface.applyCollisionObject(collision_object);  // Updates synchronously.
    planning_scene_interface.addCollisionObjects({collision_object});  // Updates asynchronously.

    // Create a new pose
    const auto target_pose = [&init_pose]() {
        geometry_msgs::msg::Pose target_pose = init_pose;
        target_pose.position.x = 0.35484119441765044;
        target_pose.position.y = -0.11281161718567351;
        target_pose.position.z = 1.0229927956818097;
        return target_pose;
    }();

    move_group_interface.setPoseTarget(target_pose);

    // Scale to 25% of the maximum joint speeds and accelerations
    move_group_interface.setMaxVelocityScalingFactor(0.25);
    move_group_interface.setMaxAccelerationScalingFactor(0.25);

    // Create a plan to that target pose
    auto const [success2, plan2] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg2;
        auto const ok2 = static_cast<bool>(move_group_interface.plan(msg2));
        return std::make_pair(ok2, msg2);
    }();

    // Execute the plan
    if (success2) {
        move_group_interface.execute(plan2);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
