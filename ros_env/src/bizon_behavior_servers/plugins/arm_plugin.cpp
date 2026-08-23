#include "bizon_behavior_servers/plugins/arm_plugin.hpp"

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "bizon_chess/board_geometry.hpp"
#include "bizon_chess/fen.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bizon_behaviors
{

ArmPlugin::ArmPlugin() = default;
ArmPlugin::~ArmPlugin() = default;

void ArmPlugin::onConfigure()
{
  auto node = node_.lock();

  if (!node->has_parameter(behavior_name_ + ".planning_time")) {
    node->declare_parameter(behavior_name_ + ".planning_time", 15.0);
  }
  node->get_parameter(behavior_name_ + ".planning_time", planning_time_);

  // Only the two RobotParams fields squareToWorldMirrored() actually reads
  // (box_size, robot_base_offset_x) are exposed here; the rest keep
  // RobotParams's defaults, matching DecisionPlugin's onConfigure() -- see
  // its comment for why calibration-only fields (link lengths, gripper
  // gaps, prismatic limits) are left alone rather than duplicated onto a
  // plugin that never uses them. mirrored is set per-goal in onRun() from
  // the goal's player_side, not read as a parameter.
  if (!node->has_parameter(behavior_name_ + ".box_size")) {
    node->declare_parameter(behavior_name_ + ".box_size", params_.box_size);
  }
  node->get_parameter(behavior_name_ + ".box_size", params_.box_size);

  if (!node->has_parameter(behavior_name_ + ".robot_base_offset_x")) {
    node->declare_parameter(behavior_name_ + ".robot_base_offset_x", params_.robot_base_offset_x);
  }
  node->get_parameter(behavior_name_ + ".robot_base_offset_x", params_.robot_base_offset_x);

  // Cylinder dimensions for each piece's collision object. Defaults are a
  // rough king-sized cylinder; real geometry is a hardware-phase
  // calibration task (see the brief's "Deferred to the Hardware Phase").
  if (!node->has_parameter(behavior_name_ + ".piece_height")) {
    node->declare_parameter(behavior_name_ + ".piece_height", piece_height_);
  }
  node->get_parameter(behavior_name_ + ".piece_height", piece_height_);

  if (!node->has_parameter(behavior_name_ + ".piece_radius")) {
    node->declare_parameter(behavior_name_ + ".piece_radius", piece_radius_);
  }
  node->get_parameter(behavior_name_ + ".piece_radius", piece_radius_);

  // See the header comment on moveit_node_: MoveGroupInterface needs a
  // plain rclcpp::Node, which the LifecycleNode we were configured with is
  // not, so we spin a dedicated helper node for it instead.
  rclcpp::NodeOptions moveit_node_options;
  moveit_node_options.automatically_declare_parameters_from_overrides(true);
  moveit_node_ = std::make_shared<rclcpp::Node>(
    behavior_name_ + "_moveit_client", node->get_namespace(), moveit_node_options);
  moveit_node_thread_ = std::make_unique<bizon_util::NodeThread>(moveit_node_);

  moveit::planning_interface::MoveGroupInterface::Options arm_options(
    "arm_group", "robot_description", node->get_namespace());
  moveit::planning_interface::MoveGroupInterface::Options hand_options(
    "hand_group", "robot_description", node->get_namespace());

  move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    moveit_node_, arm_options);
  move_group_hand_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    moveit_node_, hand_options);

  move_group_arm_->setPlanningTime(planning_time_);
  move_group_hand_->setPlanningTime(planning_time_);

  // Namespaced exactly like the two MoveGroupInterfaces above -- see the
  // header comment on planning_scene_ for why a default-constructed (global
  // namespace) instance would silently talk to no one on a namespaced robot.
  planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
    node->get_namespace());

  RCLCPP_INFO(node->get_logger(), "[%s] ArmPlugin configured", behavior_name_.c_str());
}

void ArmPlugin::onCleanup()
{
  move_group_arm_.reset();
  move_group_hand_.reset();
  planning_scene_.reset();
  moveit_node_thread_.reset();
  moveit_node_.reset();
}

ResultStatus ArmPlugin::onRun(const std::shared_ptr<const ArmAction::Goal> command)
{
  auto node = node_.lock();

  // target_hand_position is always required: even a hand_only goal has to
  // say where the gripper should go.
  if (command->target_hand_position.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty hand target in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 1};
  }

  target_hand_position_ = command->target_hand_position;

  // Task 7 (F6): publish the board and every occupied square as collision
  // objects so joint-space plans route around pieces instead of sweeping
  // through them. An empty board_fen means "leave the scene alone" -- the
  // home-position and RecoverArm goals carry no board context and are not
  // wired to this port (see chess_game.xml), so they neither clear nor
  // refresh what a preceding MoveSequence call last published.
  if (!command->board_fen.empty()) {
    // Drives bizon_chess::squareToWorldMirrored() below -- see Ruling 1 in
    // the Task 7 brief: a black-side robot mirrors both axes, and getting
    // this wrong would place every cylinder on the wrong square, routing
    // the planner around phantom pieces while it sweeps through real ones.
    params_.mirrored = (command->player_side != "white");

    // applyCollisionObjects() only ADDs/updates the objects it is given; it
    // does not remove ones that are absent from the list. Without an
    // explicit clear first, a square that was occupied on an earlier goal
    // (a capture, a promotion pickup) would keep a stale collision object
    // forever once its piece leaves -- the scene would only ever grow.
    // Removing all 64 possible ids unconditionally before re-adding the
    // currently-occupied ones is cheap and makes every rebuild an exact
    // snapshot of board_fen, regardless of what the scene held before.
    std::vector<std::string> all_square_ids;
    all_square_ids.reserve(64);
    for (char file = 'a'; file <= 'h'; ++file) {
      for (char rank = '1'; rank <= '8'; ++rank) {
        all_square_ids.push_back(std::string("piece_") + file + rank);
      }
    }
    planning_scene_->removeCollisionObjects(all_square_ids);

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (const auto & square : bizon_chess::occupiedSquares(command->board_fen)) {
      double x = 0.0;
      double y = 0.0;
      if (!bizon_chess::squareToWorldMirrored(square, params_, x, y)) {
        continue;
      }

      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = move_group_arm_->getPlanningFrame();
      obj.id = "piece_" + square;
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {piece_height_, piece_radius_};
      obj.primitives.push_back(primitive);

      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = piece_height_ / 2.0;
      obj.primitive_poses.push_back(pose);

      objects.push_back(obj);
    }
    planning_scene_->applyCollisionObjects(objects);

    // The piece this goal is about to approach must not appear as an
    // obstacle to itself, or the planner refuses to reach the square it is
    // descending onto (a pick or a place). board_fen is the last
    // camera-confirmed position and does not change mid-move (see
    // fen_pending_ in DecisionPlugin), so this object is republished by the
    // rebuild above on every later call in the same move once target_square
    // moves on to a different square -- deliberately: the source square is
    // genuinely still "occupied" as far as the rest of the scene is
    // concerned once its piece is lifted and in transit, which only makes
    // the retreat/transit path more conservative around empty space, never
    // unsafe. Attaching the carried piece to the gripper so the scene
    // tracks it in flight is real future work, not needed to close F6.
    if (!command->target_square.empty()) {
      planning_scene_->removeCollisionObjects({"piece_" + command->target_square});
    }
  }

  if (command->hand_only) {
    // The recovery subtree's first step must open the gripper to release a
    // held piece without moving the arm -- moving first would drag the
    // piece across the board. So when hand_only is set, skip the arm
    // entirely: do not validate or use target_joint_positions, do not call
    // setJointValueTarget on the arm group, do not launch an arm move.
    // future_arm_ is left untouched (default-constructed) and must never be
    // waited on or get()-ed while phase_ == HAND_MOVING was entered this way.
    RCLCPP_INFO(
      node->get_logger(), "[%s] hand_only goal: arm left stationary", behavior_name_.c_str());

    move_group_hand_->setJointValueTarget(target_hand_position_);
    future_hand_ = std::async(std::launch::async, [this]() { return move_group_hand_->move(); });
    phase_ = Phase::HAND_MOVING;

    return ResultStatus{Status::SUCCEEDED, 0};
  }

  if (command->target_joint_positions.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty joint target in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 1};
  }

  if (move_group_arm_->getCurrentJointValues().empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] no joint state available", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 2};
  }

  move_group_arm_->setJointValueTarget(command->target_joint_positions);
  future_arm_ = std::async(std::launch::async, [this]() { return move_group_arm_->move(); });
  phase_ = Phase::ARM_MOVING;

  return ResultStatus{Status::SUCCEEDED, 0};
}

ResultStatus ArmPlugin::onCycleUpdate()
{
  auto node = node_.lock();

  if (phase_ == Phase::ARM_MOVING) {
    if (future_arm_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      return ResultStatus{Status::RUNNING, 0};
    }
    const auto result = future_arm_.get();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "[%s] arm move failed: %d", behavior_name_.c_str(), result.val);
      return ResultStatus{Status::FAILED, static_cast<uint16_t>(-result.val)};
    }

    move_group_hand_->setJointValueTarget(target_hand_position_);
    future_hand_ = std::async(std::launch::async, [this]() { return move_group_hand_->move(); });
    phase_ = Phase::HAND_MOVING;
    return ResultStatus{Status::RUNNING, 0};
  }

  // phase_ == Phase::HAND_MOVING. This branch is reached either after a
  // completed arm move (future_hand_ launched just above) or directly from a
  // hand_only goal (future_hand_ launched in onRun). Either way future_arm_
  // is never touched here.
  if (future_hand_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    return ResultStatus{Status::RUNNING, 0};
  }
  const auto result = future_hand_.get();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[%s] hand move failed: %d", behavior_name_.c_str(), result.val);
    return ResultStatus{Status::FAILED, static_cast<uint16_t>(-result.val)};
  }

  return ResultStatus{Status::SUCCEEDED, 0};
}

}  // namespace bizon_behaviors

PLUGINLIB_EXPORT_CLASS(bizon_behaviors::ArmPlugin, bizon_core::Behavior)
