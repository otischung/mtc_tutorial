#pragma once
#include <rclcpp/rclcpp.hpp>

// Interface with the robot model and collision objects
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Different components of MoveIt Task Constructor that are used in the example
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

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

namespace mtc = moveit::task_constructor;

/**
 * @class MTCTaskNode
 * @brief A class that handles the creation, planning, and execution of a
 *        MoveIt Task Constructor (MTC) task.
 */
class MTCTaskNode {
   public:
    /**
     * @brief Constructor for MTCTaskNode.
     * @param options Node options for initializing the underlying ROS 2 node.
     */
    MTCTaskNode(const rclcpp::NodeOptions& options);

    /**
     * @brief Getter function to get the node base interface, which will be used for the executor later.
     * @return A shared pointer to the node base interface.
     */
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    /**
     * @brief This class method is used to set up the planning scene that is used in the example.
     * It creates a cylinder with dimensions specified by object.primitives[0].dimensions
     * and position specified by pose.position.x and pose.position.y.
     * You can try changing these numbers to resize and move the cylinder around.
     * If we move the cylinder out of the robot’s reach, planning will fail.
     */
    void setupPlanningScene();

    /**
     * @brief This function interfaces with the MoveIt Task Constructor task object.
     */
    void doTask();

   private:
    // Compose an MTC task from a series of stages.
    /**
     * @brief Creates a MoveIt Task Constructor object and sets some initial properties.
     * @return An MTC Task object configured with stages.
     */
    mtc::Task createTask();
    mtc::Task task_;                // The task object used to compose and execute MTC stages.
    rclcpp::Node::SharedPtr node_;  // Shared pointer to the underlying ROS 2 node.
};
