#include "mtc_tutorial/mtc_task_node.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)} {
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
    return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene() {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = -0.25;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask() {
    task_ = createTask();

    try {
        task_.init();
    } catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    // Generates a plan, stopping after 5 successful plans are found.
    if (!task_.plan(5)) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }

    // The next line publishes the solution to be visualized in RViz.
    // This line can be removed if you don’t care for visualization.
    task_.introspection().publishSolution(*task_.solutions().front());

    // Executes the plan.
    // Execution occurs via an action server interface with the RViz plugin.
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

mtc::Task MTCTaskNode::createTask() {
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "panda_arm";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "panda_hand";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    // Forward current_state on to grasp pose generator
    // This creates a pointer to a stage such that we can reuse stage information in specific scenarios.
    mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop

    // Make a current_state stage (a GENERATOR stage) and add it to our task.
    // This starts the robot off in its current state.
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    // Save a pointer to it in the current_state_ptr for later use.
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    /**
     * Solvers are used to define the type of robot motion.
     * MoveIt Task Constructor has three options for solvers:
     */

    // PipelinePlanner uses MoveIt’s planning pipeline, which typically defaults to Open Motion Planning Library (OMPL).
    // https://github.com/ompl/ompl
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    // JointInterpolation is a simple planner that interpolates between the start and goal joint states.
    // It is typically used for simple motions as it computes quickly but doesn’t support complex motions.
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    // CartesianPath is used to move the end effector in a straight line in Cartesian space.
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    // Feel free to try out the different solvers and see how the robot motion changes.
    // For the first stage we will use the Cartesian planner, which requires the following properties to be set:
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    // Now that we added in the planners, we can add a stage that will move the robot.
    // The following lines use a MoveTo stage (a PROPAGATOR stage).
    // Since opening the hand is a relatively simple movement, we can use the joint interpolation planner.
    // This stage plans a move to the “open hand” pose, which is a named pose defined in the SRDF for the Panda robot.
    // We return the task and finish with the createTask() function.
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    // We need to move the arm to a position where we can pick up our object. This is done with a Connect stage,
    // which as its name implies, is a CONNECTOR stage.
    // This means that it tries to bridge between the results of the stage before and after it.
    // This stage is initialized with a name, move_to_pick, and a GroupPlannerVector that specifies the planning group and the planner.
    // We then set a timeout for the stage, set the properties for the stage, and add it to our task.
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    // Next, we create a pointer to a MoveIt Task Constructor stage object, and set it to nullptr for now.
    // Later, we will use this to save a stage.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable" /*  */
    mtc::Stage* attach_object_stage = nullptr;             // Forward attach_object_stage to place pose generator
#pragma GCC diagnostic pop

    {
        // This is a container that can be added to our task and can hold several substages.
        // In this case, we create a serial container that will contain the stages relevant to the picking action.
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");

        // Instead of adding the stages to the task, we will add the relevant stages to the serial container.
        // We use exposeTo() to declare the task properties from the parent task in the new serial container, and use configureInitFrom() to initialize them.]
        // This allows the contained stages to access these properties.
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              {"eef", "group", "ik_frame"});
        {
            // We then create a stage to approach the object. This stage is a MoveRelative stage, which allows us to specify a relative movement from our current position.
            // MoveRelative is a PROPAGATOR stage: it receives the solution from its neighbouring stages and propagates it to the next or previous stage.
            // Using cartesian_planner finds a solution that involves moving the end effector in a straight line.
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);

            // We set the properties, and set the minimum and maximum distance to move.
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.15);

            // Now we create a Vector3Stamped message to indicate the direction we want to move - in this case, in the Z direction from the hand frame.
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);

            // Finally, we add this stage to our serial container
            grasp->insert(std::move(stage));
        }
        {
            // Now, create a stage to generate the grasp pose.
            // This is a GENERATOR stage, so it computes its results without regard to the stages before and after it.
            // The first stage, CurrentState is a generator stage as well - to connect the first stage and this stage,
            // a connecting stage must be used, which we already created above.
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");

            // This code sets the stage properties, sets the pose before grasping, the angle delta, and the monitored stage.
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject("object");

            // Angle delta is a property of the GenerateGraspPose stage that is used to determine the number of poses to generate;
            // when generating solutions, MoveIt Task Constructor will try to grasp the object from many different orientations,
            // with the difference between the orientations specified by the angle delta.
            // The smaller the delta, the closer together the grasp orientations will be.
            stage->setAngleDelta(M_PI / 12);

            // When defining the current stage, we set current_state_ptr, which is now used to forward information about the object pose and shape to the inverse kinematics solver.
            // This stage won’t be directly added to the serial container like previously, as we still need to do inverse kinematics on the poses it generates.
            stage->setMonitoredStage(current_state_ptr);  // Hook into current state
            
            // Before we compute inverse kinematics for the poses generated above, we first need to define the frame.
            // This can be done with a PoseStamped message from geometry_msgs or in this case, we define the transform using Eigen transformation matrix and the name of the relevant link.
            // Here, we define the transformation matrix.
            Eigen::Isometry3d grasp_frame_transform;  // 4x4 transformation matrix representing an isometry in 3D space. It includes both rotation and translation but no scaling or shearing.
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *  // First rotate pi / 2 about the x-axis
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *  // Next rotate pi / 2 about the y-axis
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());   // Finally rotate pi / 2 about the z-axis
            // q.matrix() converts the quaternion q into a 3x3 rotation matrix.
            // grasp_frame_transform.linear() accesses the rotational component of the 4x4 transformation matrix and assigns it the rotation matrix derived from q.
            grasp_frame_transform.linear() = q.matrix();
            // grasp_frame_transform.translation() accesses the translational component (a 3D vector) of the transformation matrix.
            // This sets the Z-axis translation to 0.1 units, meaning the frame is shifted upward along the Z-axis by 0.1 meters.
            grasp_frame_transform.translation().z() = 0.1;

            // Compute IK
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
    }
    return task;
}
