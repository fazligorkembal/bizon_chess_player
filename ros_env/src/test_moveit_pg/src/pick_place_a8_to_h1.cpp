#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

inline void calcula_joint_angles(float x, float y, float l1, float l2, float & q1, float & q2)
{
  float D = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  q2 = atan2f(sqrtf(1 - D * D), D);
  q1 = atan2f(y, x) - atan2f(l2 * sinf(q2), l1 + l2 * cosf(q2));
}

void pick_and_place(
  moveit::planning_interface::MoveGroupInterface & move_group,
  moveit::planning_interface::MoveGroupInterface & hand_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  moveit::planning_interface::MoveGroupInterface::Plan & gripper_plan,
  float l1, float l2,
  float box_width,
  float closed_gap,
  float from_x, float from_y,
  float to_x, float to_y)
{
  float q1, q2;
  auto joints = move_group.getCurrentJointValues();
  if (joints.size() < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_and_place"), "Joint state not received!");
    return;
  }

  std::vector<double> target = joints;
  float angle_holder = 0.0f;
  calcula_joint_angles(0.30f + (box_width * from_x), (box_width * from_y), l1, l2, q1, q2);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "IK Results: q1=%.3f, q2=%.3f", q1, q2);
  target[0] = q1;
  target[2] = q2;
  target[3] = 0.0f;
  angle_holder = (q1 + q2) + M_PI / 4.0f;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Moved to pick position");

  std::vector<double> gripper_open = {0.2, 0.2, 0.2};
  hand_group.setJointValueTarget(gripper_open);
  hand_group.plan(gripper_plan);
  hand_group.execute(gripper_plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Gripper opened");

  target[1] = 0.155f; // Prismatic joint for picking
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Lowered to pick");

  gripper_open = {closed_gap, closed_gap, closed_gap};
  hand_group.setJointValueTarget(gripper_open);
  hand_group.plan(gripper_plan);
  hand_group.execute(gripper_plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Gripper closed");
  std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Short delay before moving to place position


  target[1] = 0.06f; // Lift after picking
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Lifted after pick");

  calcula_joint_angles(0.30f + (box_width * to_x), (box_width * to_y), l1, l2, q1, q2);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "IK Results: q1=%.3f, q2=%.3f", q1, q2);
  target[0] = q1;
  target[2] = q2;
  target[3] = angle_holder;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Moved to place position");

  target[1] = 0.155f; // Prismatic joint for placing
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Lowered to place");

  gripper_open = {0.2, 0.2, 0.2};

  hand_group.setJointValueTarget(gripper_open);
  hand_group.plan(gripper_plan);
  hand_group.execute(gripper_plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Gripper opened to release");

  target[1] = 0.06f; // Lift after placing
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "Lifted after place");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  float l1 = 0.29f;
  float l2 = 0.18f;
  float box_width = 0.0371875f;
  float closed_gap = 0.04;

  auto node = rclcpp::Node::make_shared(
    "first_app_node",
    node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();

  RCLCPP_INFO(
    node->get_logger(),
    "Node namespace: %s", node->get_namespace());

  moveit::planning_interface::MoveGroupInterface::Options options(
    "arm_group",
    "robot_description",
    node->get_namespace());

  moveit::planning_interface::MoveGroupInterface::Options options_eef(
    "hand_group",
    "robot_description",
    node->get_namespace());

  moveit::planning_interface::MoveGroupInterface move_group(node, options);
  moveit::planning_interface::MoveGroupInterface hand_group(node, options_eef);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  // Velocity and acceleration settings
  move_group.setMaxVelocityScalingFactor(1.0);     // Maximum velocity
  move_group.setMaxAccelerationScalingFactor(1.0); // Maximum acceleration
  hand_group.setMaxVelocityScalingFactor(1.0);
  hand_group.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(node->get_logger(), "Velocity and acceleration set to maximum values");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto joints = move_group.getCurrentJointValues();
  if (joints.size() < 3) {
    RCLCPP_ERROR(node->get_logger(), "Joint state not received!");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Current joints: %.3f %.3f %.3f",
    joints[0], joints[1], joints[2]);

  std::vector<double> target = joints;
  auto current = move_group.getCurrentJointValues();
  RCLCPP_INFO(node->get_logger(), "Current position: %.3f", current[0]);

  move_group.setStartStateToCurrentState();

  for (int i = 0; i < 6; ++i) {
    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      3.5f, 3.5f,
      -3.5f, 3.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      3.5f, 2.5f,
      -3.5f, 2.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      3.5f, 1.5f,
      -3.5f, 1.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      3.5f, 0.5f,
      -3.5f, 0.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      3.5f, -0.5f,
      -3.5f, -0.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      3.5f, -1.5f,
      -3.5f, -1.5f);

    //////////////////////////////////////////////////////////
    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      -3.5f, 3.5f,
      3.5f, 3.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      -3.5f, 2.5f,
      3.5f, 2.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      -3.5f, 1.5f,
      3.5f, 1.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      -3.5f, 0.5f,
      3.5f, 0.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      -3.5f, -0.5f,
      3.5f, -0.5f);

    pick_and_place(
      move_group, hand_group, plan, gripper_plan,
      l1, l2, box_width, closed_gap,
      -3.5f, -1.5f,
      3.5f, -1.5f);
  }

  rclcpp::shutdown();
  return 0;
}
