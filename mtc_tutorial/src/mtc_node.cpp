#include "mtc_tutorial/mtc_node.hpp" // Include your own header first

#include <mutex>

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
  auto get_or_declare_bool = [this](const std::string &name, bool default_value) {
    if (node_->has_parameter(name)) {
      bool value = default_value;
      (void)node_->get_parameter(name, value);
      return value;
    }
    return node_->declare_parameter<bool>(name, default_value);
  };

  auto get_or_declare_string = [this](const std::string &name, const std::string &default_value) {
    if (node_->has_parameter(name)) {
      std::string value = default_value;
      (void)node_->get_parameter(name, value);
      return value;
    }
    return node_->declare_parameter<std::string>(name, default_value);
  };

  auto get_or_declare_double = [this](const std::string &name, double default_value) {
    if (node_->has_parameter(name)) {
      double value = default_value;
      (void)node_->get_parameter(name, value);
      return value;
    }
    return node_->declare_parameter<double>(name, default_value);
  };

  use_detected_object_pose_ = get_or_declare_bool("use_detected_object_pose", false);
  detected_pose_topic_ = get_or_declare_string("detected_pose_topic", "/detected_object_pose_stable");
  detection_wait_timeout_sec_ = get_or_declare_double("detection_wait_timeout_sec", 10.0);
  arm_group_name_ = get_or_declare_string("arm_group_name", "arm");
  gripper_group_name_ = get_or_declare_string("gripper_group_name", "gripper");
  gripper_frame_ = get_or_declare_string("gripper_frame", "Gripper_1");
  world_frame_ = get_or_declare_string("world_frame", "world");
  gripper_open_pose_ = get_or_declare_string("gripper_open_pose", "open");
  gripper_close_pose_ = get_or_declare_string("gripper_close_pose", "close");
  arm_home_pose_ = get_or_declare_string("arm_home_pose", "home");
  pickup_x_ = get_or_declare_double("pickup_x", pickup_x_);
  pickup_y_ = get_or_declare_double("pickup_y", pickup_y_);
  pickup_z_ = get_or_declare_double("pickup_z", pickup_z_);

  if (use_detected_object_pose_) {
    detected_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        detected_pose_topic_, 10,
        std::bind(&MTCTaskNode::detectedPoseCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
        LOGGER,
        "Detection-driven pickup enabled. topic=%s wait_timeout=%.1fs",
        detected_pose_topic_.c_str(),
        detection_wait_timeout_sec_);
  } else {
    RCLCPP_INFO(
        LOGGER,
        "Using static pickup pose x=%.3f y=%.3f z=%.3f",
        pickup_x_, pickup_y_, pickup_z_);
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = world_frame_;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // Use class constants instead of local constants
  object.primitives[0].dimensions = {OBJECT_HEIGHT, OBJECT_RADIUS};

  geometry_msgs::msg::Pose pose;
  pose.position.x = pickup_x_;
  pose.position.y = pickup_y_;
  // pose.position.z = OBJECT_TABLE_HEIGHT;
  pose.position.z = pickup_z_ + OBJECT_HEIGHT*2/3;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::detectedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(detection_mutex_);
  pickup_x_ = msg->pose.position.x;
  pickup_y_ = msg->pose.position.y;
  pickup_z_ = msg->pose.position.z;
  if (!detected_pose_ready_) {
    detected_pose_ready_ = true;
    RCLCPP_INFO(
        LOGGER,
        "Detected pickup pose accepted x=%.3f y=%.3f z=%.3f",
        pickup_x_, pickup_y_, pickup_z_);
  }
}

bool MTCTaskNode::waitForDetectedPose()
{
  if (!use_detected_object_pose_) {
    return true;
  }

  RCLCPP_INFO(
      LOGGER,
      "Waiting up to %.1f s for stable detected pose on %s",
      detection_wait_timeout_sec_,
      detected_pose_topic_.c_str());

  const auto start = node_->now();
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok()) {
    if (detected_pose_ready_) {
      return true;
    }
    if ((node_->now() - start).seconds() > detection_wait_timeout_sec_) {
      RCLCPP_ERROR(
          LOGGER,
          "Timed out waiting for stable detected pose on %s",
          detected_pose_topic_.c_str());
      return false;
    }
    rate.sleep();
  }
  return false;
}

void MTCTaskNode::doTask()
{
  if (!waitForDetectedPose()) {
    return;
  }

  setupPlanningScene(); // Ensure object exists before planning
  RCLCPP_INFO(
      LOGGER,
      "Planning with pickup pose x=%.3f y=%.3f z=%.3f",
      pickup_x_, pickup_y_, pickup_z_);
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

  const auto &arm_group_name = arm_group_name_;
  const auto &hand_group_name = gripper_group_name_;
  const auto &hand_frame = gripper_frame_;

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.setProperty("velocity_scaling_factor", 0.2);
  task.setProperty("acceleration_scaling_factor", 0.2);

  task.setProperty("max_velocity_scaling_factor", 0.2);
  task.setProperty("max_acceleration_scaling_factor", 0.2);

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

  // Also apply to your sampling planner (PipelinePlanner)
  sampling_planner->setProperty("max_velocity_scaling_factor", 0.2);
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.2);
  interpolation_planner->setMaxVelocityScalingFactor(0.2);
  interpolation_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setMaxVelocityScalingFactor(0.2); // slower
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);

  // sampling_planner->setProperty("velocity_scaling_factor", 0.2);
  // sampling_planner->setProperty("acceleration_scaling_factor", 0.2);

  cartesian_planner->setStepSize(0.01);
  cartesian_planner->setMinFraction(1.0);

  // ===== OPEN HAND =====
  {
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    // Slow down gripper motion for smooth open
    stage_open_hand->setProperty("max_velocity_scaling_factor", 0.05);
    stage_open_hand->setProperty("max_acceleration_scaling_factor", 0.05);

    // Explicitly tell MTC which hardware controller to use
    moveit::task_constructor::TrajectoryExecutionInfo exec_info;
    exec_info.set__controller_names({"gripper_controller"});
    // Apply to a stage
    stage_open_hand->properties().set("trajectory_execution_info", exec_info);

    stage_open_hand->setGoal(gripper_open_pose_);
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
    stage_move_to_pick->setProperty("max_velocity_scaling_factor", 0.2);
    stage_move_to_pick->setProperty("max_acceleration_scaling_factor", 0.2);

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
      stage->properties().set("max_velocity_scaling_factor", 0.2); // 接近物體時減速
      stage->properties().set("max_acceleration_scaling_factor", 0.2);

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
      stage->setPreGraspPose(gripper_open_pose_);
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

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(0.1);
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().y() = GRIPPER_JAW_OFFSET_Y;
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
      // Keep grasp candidates where the long cylinder briefly intersects forearm links.
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(arm_group_name)
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
      // Slow down gripper motion for smooth close
      stage->setProperty("max_velocity_scaling_factor", 0.02);
      stage->setProperty("max_acceleration_scaling_factor", 0.02);
      stage->setGoal(gripper_close_pose_);
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
      vec.header.frame_id = world_frame_;
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
    stage_move_to_place->setTimeout(15.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_place->setProperty("max_velocity_scaling_factor", 0.2);
    stage_move_to_place->setProperty("max_acceleration_scaling_factor", 0.2);

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
      // Slow down gripper motion for smooth open
      stage->setProperty("max_velocity_scaling_factor", 0.05);
      stage->setProperty("max_acceleration_scaling_factor", 0.05);
      stage->setGoal(gripper_open_pose_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
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

    // Forbid hand-object collision after retreat to avoid filtering valid release states.
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

    task.add(std::move(place));
  }

  // ===== RETURN HOME =====
  {
    // auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    // Explicitly tell MTC which hardware controller to use
    moveit::task_constructor::TrajectoryExecutionInfo exec_info;
    exec_info.set__controller_names({"arm_controller"});
    // Apply to a stage
    stage->properties().set("trajectory_execution_info", exec_info);
    stage->setProperty("max_velocity_scaling_factor", 0.2);
    stage->setProperty("max_acceleration_scaling_factor", 0.2);
    stage->setGroup(arm_group_name);
    stage->setGoal(arm_home_pose_);
    stage->setTimeout(15.0);
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

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
