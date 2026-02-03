#ifndef MTC_TUTORIAL_MTC_NODE_HPP
#define MTC_TUTORIAL_MTC_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>

namespace mtc = moveit::task_constructor;

/**
 * @class MTCTaskNode
 * @brief Implements pick-and-place task using Motion Task Constructor
 *
 * Manages object pickup, transport, and placement for Arctos manipulator.
 */
class MTCTaskNode
{
public:
  /**
   * @brief Constructor
   * @param options ROS node options
   */
  explicit MTCTaskNode(const rclcpp::NodeOptions &options);

  /**
   * @brief Get node base interface for executor integration
   * @return Shared pointer to node base interface
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  /**
   * @brief Execute pick-and-place task
   *
   * Plans and executes the full manipulation task:
   * 1. Move to pickup location
   * 2. Grasp object with orientation sampling
   * 3. Lift object
   * 4. Move to place location
   * 5. Release object and retreat
   * 6. Return to home
   */
  void doTask();

  /**
   * @brief Setup planning scene with collision objects
   *
   * Adds cylinder object at pickup location for collision checking.
   */
  void setupPlanningScene();

  // =========== ROBOT GEOMETRY CONSTANTS ===========
  /// Object to be manipulated: cylinder radius (meters)
  static constexpr double OBJECT_RADIUS = 0.01; // 1 cm radius

  /// Gripper maximum jaw opening (meters)
  static constexpr double GRIPPER_WIDTH = 0.03; // 3 cm max

  /// Distance from Gripper_1 frame to jaw center (meters)
  static constexpr double GRIPPER_JAW_OFFSET_Y = -0.06;

  /// Object height (meters)
  static constexpr double OBJECT_HEIGHT = 0.2;

  /// Vertical offset of object from table (half cylinder height, meters)
  static constexpr double OBJECT_TABLE_HEIGHT = 0.14;

  // =========== PLANNING SCENE PARAMETERS ===========
  /// X position of pickup location (meters)
  static constexpr double PICKUP_X = 0.32;

  /// Y position of pickup location (meters)
  static constexpr double PICKUP_Y = -0.36;

  /// Minimum grasp sampling angle delta (radians)
  static constexpr double MIN_ANGLE_DELTA = M_PI / 36; // 5°

  // =========== PLACE PARAMETERS ===========
  /// X position of place location relative to object frame (meters)
  static constexpr double PLACE_OFFSET_X = -0.65; // or 0

  /// Y position of place location relative to object frame (meters)
  static constexpr double PLACE_OFFSET_Y = -0.13;

private:
  /// Compose MTC task from series of stages
  mtc::Task createTask();

  /// MTC task instance
  mtc::Task task_;

  /// ROS 2 node
  rclcpp::Node::SharedPtr node_;
};

#endif // MTC_TUTORIAL_MTC_NODE_HPP