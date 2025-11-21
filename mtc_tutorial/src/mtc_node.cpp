#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
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
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
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
  object.primitives[0].dimensions = {0.1, 0.02};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5; // Closer to robot
  pose.position.y = -0.25; // Centered
  pose.position.z = 0.42; // Raised height
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();
  task_.setProperty("action_ns", "/execute_task_solution");

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  // 5s
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  RCLCPP_INFO_STREAM(LOGGER, "Task planning succeeded! Found " << task_.solutions().size() << " solutions.");
  // pub 1st planning solution
  task_.introspection().publishSolution(*task_.solutions().front());

  // Check if execution action server is available
  RCLCPP_INFO_STREAM(LOGGER, "Checking for action server...");

  // Wait a bit longer for action server
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Try to execute - if it fails, just continue with visualization
  try
  {
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_WARN_STREAM(LOGGER, "Task execution failed with code " << result.val << " - solution available for visualization");
      return;
    }
    RCLCPP_INFO_STREAM(LOGGER, "Task executed successfully!");
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Execution failed: " << e.what() << " - solution available for visualization");
  }

  return;
}

// all the stages are built here
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  // for ik
  task.loadRobotModel(node_);

  const auto &arm_group_name = "arm";
  const auto &hand_group_name = "gripper";
  const auto &hand_frame = "tool_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name); // Use the actual end effector name from SRDF (typo in SRDF)
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // for moving aroud the obj
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  // planner for open and close gripper coz it is faster
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  // for motion btw approach and grasp as well as lift obj
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // connector stage to move to object
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
  stage_move_to_pick->setTimeout(6.0);
  // configure properties from parent task
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // for attach obj stage
  mtc::Stage *attach_object_stage = nullptr;

  // pick container
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      // 10~15cm away from pick pose
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      // Sample grasp pose (generate pick pose)
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);              // Larger angle delta for more poses
      stage->setMonitoredStage(current_state_ptr); // hook into current state for ik

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.0; // 0.1; // moving 10cm back

      // Compute IK with more attempts
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);        // Even more IK attempts
      wrapper->setMinSolutionDistance(0.2); // Allow closer solutions

      wrapper->setMaxIKSolutions(32);        // increase IK attempts
// +     wrapper->setMinSolutionDistance(0.05); // allow closer solutions

      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

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

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      // lift 10~30cm from the current pose of the hand in the world frame
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0; // lift in z direction in the world frame
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  /* place stages commented out
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                 {hand_group_name, interpolation_planner}});
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  // Serail container for place
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "group", "ik_frame"});
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
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
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }
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
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  */

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