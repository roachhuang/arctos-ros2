#include "mtc_tutorial/mtc_node.hpp" // Include your own header first
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // Use class constants instead of local constants
  object.primitives[0].dimensions = {OBJECT_HEIGHT, OBJECT_RADIUS};

  geometry_msgs::msg::Pose pose;
  pose.position.x = PICKUP_X;
  pose.position.y = PICKUP_Y;
  pose.position.z = OBJECT_TABLE_HEIGHT;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  setupPlanningScene(); // Ensure object exists before planning
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  // Increase planning time for real-world kinematic complexity
  if (!task_.plan(10))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "arm";
  const auto &hand_group_name = "gripper";
  const auto &hand_frame = "Gripper_1";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage *current_state_ptr = nullptr;
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.02); // slower
  cartesian_planner->setMaxAccelerationScalingFactor(0.02);
  
  // Also apply to your sampling planner (PipelinePlanner)
  sampling_planner->setProperty("velocity_scaling_factor", 0.05);
  sampling_planner->setProperty("acceleration_scaling_factor", 0.05);

  cartesian_planner->setStepSize(0.01);
  cartesian_planner->setMinFraction(0.0);

  // ===== OPEN HAND =====
  {
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);

    // Explicitly tell MTC which hardware controller to use
    moveit::task_constructor::TrajectoryExecutionInfo exec_info;
    exec_info.set__controller_names({"gripper_controller"});
    // Apply to a stage
    stage_open_hand->properties().set("trajectory_execution_info", exec_info);

    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
  }

  // ===== ALLOW COLLISIONS =====
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (arm,hand,object)");
    stage->allowCollisions("object",
                           task.getRobotModel()
                               ->getJointModelGroup(arm_group_name)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    stage->allowCollisions("object",
                           task.getRobotModel()
                               ->getJointModelGroup(hand_group_name)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    task.add(std::move(stage));
  }

  // ===== MOVE TO PICK =====
  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);

    // Explicitly tell MTC which hardware controller to use
    moveit::task_constructor::TrajectoryExecutionInfo exec_info;
    exec_info.set__controller_names({"arm_controller"});
    // Apply to a stage
    stage_move_to_pick->properties().set("trajectory_execution_info", exec_info);

    task.add(std::move(stage_move_to_pick));
  }

  mtc::Stage *attach_object_stage = nullptr;

  // ===== PICK CONTAINER =====
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach object
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.0, 0.15);

      // Explicitly tell MTC which hardware controller to use
      moveit::task_constructor::TrajectoryExecutionInfo exec_info;
      exec_info.set__controller_names({"arm_controller"});
      // Apply to a stage
      stage->properties().set("trajectory_execution_info", exec_info);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.y = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // Generate grasp pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");

      // Adaptive angle sampling based on object and gripper geometry
      // double angle_delta = 2.0 * std::atan2(GRIPPER_WIDTH, OBJECT_RADIUS);
      // stage->setAngleDelta(std::min(angle_delta, MIN_ANGLE_DELTA));
      stage->setAngleDelta(MIN_ANGLE_DELTA);
      stage->setMonitoredStage(current_state_ptr);

      // Explicitly tell MTC which hardware controller to use
      moveit::task_constructor::TrajectoryExecutionInfo exec_info;
      exec_info.set__controller_names({"arm_controller"});
      // Apply to a stage
      stage->properties().set("trajectory_execution_info", exec_info);

      // Grasp frame transform: rotate to point downward
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().y() = GRIPPER_JAW_OFFSET_Y;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      wrapper->setIgnoreCollisions(true);
      grasp->insert(std::move(wrapper));
    }

    // Allow hand-object collision
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

    // Close hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);

      // Explicitly tell MTC which hardware controller to use
      moveit::task_constructor::TrajectoryExecutionInfo exec_info;
      exec_info.set__controller_names({"gripper_controller"});
      // Apply to a stage
      stage->properties().set("trajectory_execution_info", exec_info);

      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      grasp->insert(std::move(stage));
    }

    // Attach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Lift object
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Explicitly tell MTC which hardware controller to use
      moveit::task_constructor::TrajectoryExecutionInfo exec_info;
      exec_info.set__controller_names({"arm_controller"});
      // Apply to a stage
      stage->properties().set("trajectory_execution_info", exec_info);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ===== MOVE TO PLACE =====
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);

    // Explicitly tell MTC which hardware controller to use
    moveit::task_constructor::TrajectoryExecutionInfo exec_info;
    exec_info.set__controller_names({"arm_controller"});
    // Apply to a stage
    stage_move_to_place->properties().set("trajectory_execution_info", exec_info);

    task.add(std::move(stage_move_to_place));
  }

  // ===== PLACE CONTAINER =====
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.x = PLACE_OFFSET_X;
      target_pose_msg.pose.position.y = PLACE_OFFSET_Y;

      // Correct quaternion for 180° rotation around X-axis
      tf2::Quaternion q;
      q.setRotation(tf2::Vector3(1, 0, 0), M_PI);
      target_pose_msg.pose.orientation = tf2::toMsg(q);

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(18);
      wrapper->setMinSolutionDistance(0.01);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // Open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);

      // Explicitly tell MTC which hardware controller to use
      moveit::task_constructor::TrajectoryExecutionInfo exec_info;
      exec_info.set__controller_names({"gripper_controller"});
      // Apply to a stage
      stage->properties().set("trajectory_execution_info", exec_info);

      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      place->insert(std::move(stage));
    }

    // Forbid hand-object collision
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

    // Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    // Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);

      // Explicitly tell MTC which hardware controller to use
      moveit::task_constructor::TrajectoryExecutionInfo exec_info;
      exec_info.set__controller_names({"arm_controller"});
      // Apply to a stage
      stage->properties().set("trajectory_execution_info", exec_info);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.02, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
      stage->properties().set("link", hand_frame);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame; // Use world frame for consistent retreat
      vec.vector.y = 1;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // ===== RETURN HOME =====
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);

    // Explicitly tell MTC which hardware controller to use
    moveit::task_constructor::TrajectoryExecutionInfo exec_info;
    exec_info.set__controller_names({"arm_controller"});
    // Apply to a stage
    stage->properties().set("trajectory_execution_info", exec_info);

    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}