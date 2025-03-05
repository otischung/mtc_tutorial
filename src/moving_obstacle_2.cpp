#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/**
 * @brief A ROS2 node that publishes a moving collision object to the "/collision_object" topic.
 *
 * This node creates and publishes a moveit_msgs::msg::CollisionObject message representing a box.
 * The box's y-axis position oscillates between -0.5 and +0.5 meters at a configurable speed (default: 0.5 m/s).
 * The node is named "moving_box_publisher_2".
 *
 * Example:
 *   ros2 run <package_name> moving_obstacle_2 --ros-args -p speed:=0.3
 */
class MovingBoxPublisher : public rclcpp::Node {
   public:
    /**
     * @brief Construct a new MovingBoxPublisher object.
     *
     * Initializes the node, declares the "speed" parameter, sets up the publisher on the
     * "/collision_object" topic, and creates a timer callback that periodically updates the box's position.
     */
    MovingBoxPublisher()
        : Node("moving_box_publisher_2"), y_pos_(0.0), y_dir_(1.0) {
        // Declare the "speed" parameter with default value 0.5 m/s.
        this->declare_parameter<double>("speed", 0.5);
        this->get_parameter("speed", speed_);

        // Create a publisher for CollisionObject on the "/collision_object" topic.
        publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("/collision_object", 10);

        // Timer callback updates every 10ms.
        timer_ = this->create_wall_timer(10ms, std::bind(&MovingBoxPublisher::timer_callback, this));
    }

   private:
    /**
     * @brief Timer callback to update and publish the collision object's position.
     *
     * This callback updates the y-axis position of the box based on the "speed" parameter and a 10ms time step.
     * When the y position reaches +0.5 or -0.5 meters, the direction of movement is reversed.
     * It then builds a CollisionObject message with the updated position and publishes it.
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

        // Build the CollisionObject message.
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.stamp = this->now();
        collision_object.header.frame_id = "base_link";  // Adjust if necessary.
        collision_object.id = "box1";

        // Set the overall pose of the collision object (identity).
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

        // Publish the collision object message.
        publisher_->publish(collision_object);
        RCLCPP_INFO(this->get_logger(), "Published moving box at y = %.3f (speed=%.3f m/s)", y_pos_, speed_);
    }

    rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double y_pos_;  ///< Current y-axis position of the box.
    double y_dir_;  ///< Direction of movement along the y-axis (1.0 for positive, -1.0 for negative).
    double speed_;  ///< Speed of the box movement in m/s.
};

/**
 * @brief Main entry point for the moving box publisher node.
 *
 * Initializes the ROS2 node, creates an instance of the MovingBoxPublisher class, and spins
 * the node until a shutdown signal is received.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status code.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovingBoxPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
