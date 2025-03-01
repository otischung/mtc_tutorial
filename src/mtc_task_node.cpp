#include "mtc_tutorial/mtc_task_node.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
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
    } catch (mtc::InitStageException &e) {
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

    const auto &arm_group_name = "panda_arm";
    const auto &hand_group_name = "hand";
    const auto &hand_frame = "panda_hand";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    // Forward current_state on to grasp pose generator
    // This creates a pointer to a stage such that we can reuse stage information in specific scenarios.
    mtc::Stage *current_state_ptr = nullptr;
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
    mtc::Stage *attach_object_stage = nullptr;             // Forward attach_object_stage to place pose generator
#pragma GCC diagnostic pop

    /***********************
     * Generate Pick Stages
     ***********************/
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
            Eigen::Isometry3d grasp_frame_transform;                                        // 4x4 transformation matrix representing an isometry in 3D space. It includes both rotation and translation but no scaling or shearing.
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *  // First rotate pi / 2 about the x-axis
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *  // Next rotate pi / 2 about the y-axis
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());   // Finally rotate pi / 2 about the z-axis
            // q.matrix() converts the quaternion q into a 3x3 rotation matrix.
            // grasp_frame_transform.linear() accesses the rotational component of the 4x4 transformation matrix and assigns it the rotation matrix derived from q.
            grasp_frame_transform.linear() = q.matrix();
            // grasp_frame_transform.translation() accesses the translational component (a 3D vector) of the transformation matrix.
            // This sets the Z-axis translation to 0.1 units, meaning the frame is shifted upward along the Z-axis by 0.1 meters.
            grasp_frame_transform.translation().z() = 0.1;

            // Now, we create the ComputeIK stage, and give it the name generate pose IK as well as the generate grasp pose stage defined above.
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            // Some robots have multiple inverse kinematics solutions for a given pose - we set the limit on the amount of solutions to solve for up to 8.
            wrapper->setMaxIKSolutions(8);
            // We also set the minimum solution distance, which is a threshold on how different solutions must be:
            // if the joint positions in a solution are too similar to a previous solution, it will be marked as invalid.
            wrapper->setMinSolutionDistance(1.0);
            // Next, we configure some additional properties, and add the ComputeIK stage to the serial container.
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
        // To pick up the object, we must allow collision between the hand and the object. This can be done with a ModifyPlanningScene stage.
        // The allowCollisions function lets us specify which collisions to disable. allowCollisions can be used with a container of names,
        // so we can use getLinkModelNamesWithCollisionGeometry to get all the names of links with collision geometry in the hand group.
        {
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions("object",
                                   task.getRobotModel()
                                       ->getJointModelGroup(hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
            grasp->insert(std::move(stage));
        }
        // With collisions allowed, we now can close the hand. This is done with a MoveTo stage,
        // similarly to the open hand stage from above, except moving to the close position as defined in the SRDF.
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }
        // We now use a ModifyPlanningScene stage again, this time to attach the object to the hand using attachObject.
        // Similarly to what we did with the current_state_ptr, we get a pointer to this stage for later use when generating the place pose for the object.
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }
        // Next, we lift the object with a MoveRelative stage, similarly to the approach_object stage.
        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        // With this, we have all the stages needed to pick the object. Now, we add the serial container (with all its substages) to the task.
        // If you build the package as-is, you can see the robot plan to pick up the object.
        task.add(std::move(grasp));
    }
    /*******************************
     * Connect Pick and Place Stages
     *******************************/
    // Now that the stages that define the pick are complete, we move on to defining the stages for placing the object.
    // Picking up where we left off, we add a Connect stage to connect the two, as we will soon be using a generator stage to generate the pose for placing the object.
    {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                     {hand_group_name, interpolation_planner}});
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
    }
    /************************
     * Generate Place Stages
     ************************/
    // We also create a serial container for the place stages. This is done similarly to the pick serial container.
    // The next stages will be added to the serial container rather than the task.
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT,
                                              {"eef", "group", "ik_frame"});
        {
            // This next stage generates the poses used to place the object and compute the inverse kinematics for those poses
            // it is somewhat similar to the generate grasp pose stage from the pick serial container.
            // We start by creating a stage to generate the poses and inheriting the task properties.
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject("object");

            // We specify the pose where we want to place the object with a PoseStamped message from geometry_msgs
            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = "object";
            // in this case, we choose y = 0.5 in the "object" frame.
            target_pose_msg.pose.position.y = 0.5;
            target_pose_msg.pose.orientation.w = 1.0;
            // We then pass the target pose to the stage with setPose.
            stage->setPose(target_pose_msg);
            // Next, we use setMonitoredStage and pass it the pointer to the attach_object stage from earlier.
            // This allows the stage to know how the object is attached.
            stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

            // We then create a ComputeIK stage and pass it our GeneratePlacePose stage - the rest follows the same logic as above with the pick stages.
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("object");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(wrapper));
        }
        // Now that we’re ready to place the object, we open the hand with MoveTo stage and the joint interpolation planner.
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }
        // We also can re-enable collisions with the object now that we no longer need to hold it.
        // This is done using allowCollisions almost exactly the same way as disabling collisions, except setting the last argument to false rather than true.
        {
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions("object",
                                   task.getRobotModel()
                                       ->getJointModelGroup(hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   false);
            place->insert(std::move(stage));
        }
        // Now, we can detach the object using detachObject.
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("object", hand_frame);
            place->insert(std::move(stage));
        }
        // We retreat from the object using a MoveRelative stage, which is done similarly to the approach object and lift object stages.
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "retreat");

            // Set retreat direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.x = -0.5;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        // We finish our place serial container and add it to the task.
        task.add(std::move(place));
    }
    // The final step is to return home: we use a MoveTo stage and pass it the goal pose of ready, which is a pose defined in the Panda SRDF.
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setGoal("ready");
        task.add(std::move(stage));
    }
    return task;
}
