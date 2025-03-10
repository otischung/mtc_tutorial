// This is just a test program, and it can't work properly.
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");
namespace mtc = moveit::task_constructor;

class MTCTaskNode {
   public:
    MTCTaskNode(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

   private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

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
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.284, 1.3, 1.1};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.36 + object.primitives[0].dimensions[0] / 2.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0 + object.primitives[0].dimensions[2] / 2.0;
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

    if (!task_.plan(5)) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

mtc::Task MTCTaskNode::createTask() {
    mtc::Task task;
    task.stages()->setName("piano task");
    task.loadRobotModel(node_);

    const auto &arm_group_name = "right_arm";
    const auto &hand_group_name = "right_hand";
    const auto &hand_frame = "right_hand_1";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // Stage 1: Capture the current state.
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    mtc::Stage *current_state_ptr = current_state.get();
    task.add(std::move(current_state));

    // ----- Move to Init Pose -----
    // Use a SerialContainer to compute an IK solution from an absolute init pose.
    auto move_to_init = std::make_unique<mtc::SerialContainer>("move to init");
    task.properties().exposeTo(move_to_init->properties(), {"group", "eef", "ik_frame"});
    move_to_init->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef", "ik_frame"});

    // GeneratePose stage: provide the absolute init pose.
    auto gen_pose = std::make_unique<mtc::stages::GeneratePose>("generate init pose");
    geometry_msgs::msg::PoseStamped init_pose_msg;
    init_pose_msg.header.frame_id = task.getRobotModel()->getModelFrame();
    init_pose_msg.pose.position.x = 0.3178071837760669;
    init_pose_msg.pose.position.y = -0.10556869093811046;
    init_pose_msg.pose.position.z = 1.1215113837188624;
    init_pose_msg.pose.orientation.x = -0.1345047736672149;
    init_pose_msg.pose.orientation.y = -0.1247301629367219;
    init_pose_msg.pose.orientation.z = 0.6927253372844873;
    init_pose_msg.pose.orientation.w = 0.697482945596954;
    gen_pose->setPose(init_pose_msg);
    gen_pose->setMonitoredStage(current_state_ptr);

    // ComputeIK stage: compute a valid joint configuration for the init pose.
    auto compute_ik = std::make_unique<mtc::stages::ComputeIK>("compute IK for init", std::move(gen_pose));
    compute_ik->setMaxIKSolutions(4);
    compute_ik->setIKFrame(hand_frame);
    compute_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef"});
    move_to_init->insert(std::move(compute_ik));

    // MoveTo stage: execute the computed solution using a JointInterpolationPlanner.
    auto interp_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto move = std::make_unique<mtc::stages::MoveTo>("move to init", interp_planner);
    move->setTimeout(5.0);
    // The special goal "current_solution" indicates to use the solution computed by the previous stage.
    move->setGoal("current_solution");
    move_to_init->insert(std::move(move));

    // Add the "move to init" container to the task.
    task.add(std::move(move_to_init));

    // ----- Relative Move to Target Pose -----
    // Now that the robot is at the init pose, perform a relative Cartesian move to the target pose.
    // The target is defined as a translation of -0.4 m in y from the init pose.
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.25);
    cartesian_planner->setMaxAccelerationScalingFactor(0.25);
    cartesian_planner->setStepSize(0.01);
    auto move_relative = std::make_unique<mtc::stages::MoveRelative>("move to target", cartesian_planner);
    move_relative->setTimeout(5.0);
    move_relative->setMinMaxDistance(0.4, 0.4);
    move_relative->setIKFrame(hand_frame);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = task.getRobotModel()->getModelFrame();
    vec.vector.x = 0.0;
    vec.vector.y = -0.4;
    vec.vector.z = 0.0;
    move_relative->setDirection(vec);

    task.add(std::move(move_relative));

    return task;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}
