#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("manual_planning_scene_publisher");

    // Create a publisher for the /planning_scene topic (message type: PlanningScene)
    auto publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Give the publisher some time to connect
    rclcpp::sleep_for(1s);

    // Create the PlanningScene message
    moveit_msgs::msg::PlanningScene planning_scene_msg;

    // Set the basic fields (here we leave most as empty/default)
    planning_scene_msg.name = "";
    // The robot_state here is left empty; we mark its diff flag false.
    planning_scene_msg.robot_state.is_diff = false;
    planning_scene_msg.robot_model_name = "";
    // fixed_frame_transforms, allowed_collision_matrix, link_padding, link_scale, object_colors are left empty

    // --- Now fill the world field with our collision object ---
    // Create a CollisionObject representing an obstacle
    moveit_msgs::msg::CollisionObject collision_object;
    // Fill header: use the current time and set the reference frame (e.g., "base_link")
    collision_object.header.stamp = node->now();
    collision_object.header.frame_id = "base_link";
    // Set an overall pose for the object (typically identity, so that the primitives are defined relative to this pose)
    collision_object.pose.position.x = 0.0;
    collision_object.pose.position.y = 0.0;
    collision_object.pose.position.z = 0.0;
    collision_object.pose.orientation.x = 0.0;
    collision_object.pose.orientation.y = 0.0;
    collision_object.pose.orientation.z = 0.0;
    collision_object.pose.orientation.w = 1.0;

    // Give the object an ID
    collision_object.id = "box1";

    // Leave type empty (no specific database key)
    collision_object.type.key = "";
    collision_object.type.db = "";

    // Define a solid primitive representing a box.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;  // BOX is typically represented by an enum value (often 1)
    primitive.dimensions.resize(3);
    // Set the box dimensions (example values: 0.284 m x 1.3 m x 1.1 m)
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.284;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 1.3;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 1.1;

    // Append the primitive to the collision object
    collision_object.primitives.push_back(primitive);

    // Define the pose of the primitive relative to the collision object's pose.
    geometry_msgs::msg::Pose primitive_pose;
    primitive_pose.position.x = 0.502;
    primitive_pose.position.y = 0.0;
    primitive_pose.position.z = 0.55;
    primitive_pose.orientation.x = 0.0;
    primitive_pose.orientation.y = 0.0;
    primitive_pose.orientation.z = 0.0;
    primitive_pose.orientation.w = 1.0;
    collision_object.primitive_poses.push_back(primitive_pose);

    // Meshes, planes, and subframes are left empty.

    // Set the operation. Use ADD (defined as 0) to add the object.
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Now, insert the collision object into the PlanningSceneWorld.
    planning_scene_msg.world.collision_objects.push_back(collision_object);

    // We leave the octomap field empty for this example.

    // Mark the planning scene as a diff (i.e. an incremental update)
    planning_scene_msg.is_diff = true;

    // Publish the message to the /planning_scene topic
    publisher->publish(planning_scene_msg);
    RCLCPP_INFO(node->get_logger(), "Published planning scene with collision object 'box1'.");

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
