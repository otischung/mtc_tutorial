#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/**
 * @brief A ROS2 node that publishes a dynamic planning scene with a moving collision object.
 *
 * This node continuously publishes a PlanningScene message to the "/planning_scene" topic.
 * It creates a CollisionObject (a box) that moves along the y-axis between -0.5 and +0.5 meters
 * at a speed specified by the "speed" parameter (default 0.1 m/s). The update is performed
 * periodically using a timer callback.
 */
class MovingBoxPublisher : public rclcpp::Node {
   public:
    /**
     * @brief Construct a new MovingBoxPublisher object.
     *
     * Initializes the publisher and timer, declares and gets the "speed" parameter,
     * and sets the initial y-axis position and movement direction.
     */
    MovingBoxPublisher()
        : Node("moving_box_publisher"),
          y_pos_(0.0),
          y_dir_(1.0) {
        // Declare the "speed" parameter with default value 0.5 m/s
        this->declare_parameter<double>("speed", 0.5);
        this->get_parameter("speed", speed_);

        publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

        // Timer now updates every 10 ms.
        timer_ = this->create_wall_timer(10ms, std::bind(&MovingBoxPublisher::timer_callback, this));
    }

   private:
    /**
     * @brief Timer callback that updates the box's position and publishes the planning scene.
     *
     * This function updates the y-coordinate of the box based on the current speed (from the "speed" parameter)
     * and time step (dt = 0.01 s). When the box reaches +0.5 m or -0.5 m along the y-axis, the direction reverses.
     * It constructs a PlanningScene message containing a CollisionObject (the box) with the updated y position,
     * and publishes it asynchronously.
     */
    void timer_callback() {
        // dt is 0.01 sec (10ms)
        double dt = 0.01;
        y_pos_ += y_dir_ * speed_ * dt;
        if (y_pos_ >= 0.5) {
            y_pos_ = 0.5;
            y_dir_ = -1.0;
        } else if (y_pos_ <= -0.5) {
            y_pos_ = -0.5;
            y_dir_ = 1.0;
        }

        // Build the PlanningScene message.
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.name = "";
        planning_scene_msg.robot_state.is_diff = false;
        planning_scene_msg.robot_model_name = "";

        // Build the collision object (box).
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.stamp = this->now();
        collision_object.header.frame_id = "base_link";  // Adjust if necessary.
        collision_object.id = "box1";

        // Overall pose of the collision object (identity).
        collision_object.pose.position.x = 0.0;
        collision_object.pose.position.y = 0.0;
        collision_object.pose.position.z = 0.0;
        collision_object.pose.orientation.x = 0.0;
        collision_object.pose.orientation.y = 0.0;
        collision_object.pose.orientation.z = 0.0;
        collision_object.pose.orientation.w = 1.0;

        // Define a box primitive.
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.284;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 1.3;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 1.1;
        collision_object.primitives.push_back(primitive);

        // Define the pose of the primitive relative to the object's frame.
        geometry_msgs::msg::Pose primitive_pose;
        primitive_pose.position.x = 0.502;
        primitive_pose.position.y = y_pos_;  // Updated y position.
        primitive_pose.position.z = 0.55;
        primitive_pose.orientation.x = 0.0;
        primitive_pose.orientation.y = 0.0;
        primitive_pose.orientation.z = 0.0;
        primitive_pose.orientation.w = 1.0;
        collision_object.primitive_poses.push_back(primitive_pose);

        // Set the operation to ADD.
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Insert the collision object into the world field of the planning scene.
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;

        // Publish the planning scene message.
        publisher_->publish(planning_scene_msg);
        RCLCPP_INFO(this->get_logger(), "Published moving box at y = %.3f (speed=%.3f m/s)", y_pos_, speed_);
    }

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double y_pos_;  ///< Current y-axis position of the box.
    double y_dir_;  ///< Direction of movement along the y-axis (1.0 for positive, -1.0 for negative).
    double speed_;  ///< Speed of the box movement in m/s (adjustable via parameter).
};

/**
 * @brief Main entry point for the moving box publisher node.
 *
 * Initializes the ROS2 node, creates a MovingBoxPublisher instance, and spins
 * the node until shutdown.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status code.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovingBoxPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
