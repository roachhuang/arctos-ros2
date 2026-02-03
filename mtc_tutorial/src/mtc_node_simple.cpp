#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  
  RCLCPP_INFO_STREAM(LOGGER, "Task planning succeeded! Found " << task_.solutions().size() << " solutions.");
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO_STREAM(LOGGER, "Solution published for visualization");
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("simple demo");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", "arm");
  task.setProperty("eef", "girpper");
  task.setProperty("ik_frame", "tool_link");

  // Current state
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(current_state));

  // Open gripper
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  open_hand->setGroup("gripper");
  open_hand->setGoal("open");
  task.add(std::move(open_hand));

  return task;
}

int main(int argc, char** argv)
{
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

  mtc_task_node->doTask();

  RCLCPP_INFO_STREAM(LOGGER, "Task completed. Press Ctrl+C to exit.");
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}