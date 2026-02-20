#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

inline void calcula_joint_angles(float x, float y, float l1, float l2, float &q1, float &q2)
{
  float D = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  q2 = atan2f(sqrtf(1 - D * D), D);
  q1 = atan2f(y, x) - atan2f(l2 * sinf(q2), l1 + l2 * cosf(q2));
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  float l1 = 0.29f;
  float l2 = 0.18f;

  auto node = rclcpp::Node::make_shared(
      "first_app_node",
      node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  RCLCPP_INFO(node->get_logger(),
              "Node namespace: %s", node->get_namespace());

  moveit::planning_interface::MoveGroupInterface::Options options(
      "arm_group",
      "robot_description",
      node->get_namespace()
  );

  moveit::planning_interface::MoveGroupInterface::Options options_eef(
      "gripper_group",
      "robot_description",
      node->get_namespace()
  );

  moveit::planning_interface::MoveGroupInterface move_group(node, options);
  moveit::planning_interface::MoveGroupInterface gripper_group(node, options_eef);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  // Hız ve ivme ayarları
  move_group.setMaxVelocityScalingFactor(1.0);  // Maksimum hız
  move_group.setMaxAccelerationScalingFactor(1.0);  // Maksimum ivme
  gripper_group.setMaxVelocityScalingFactor(1.0);
  gripper_group.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(node->get_logger(), "Hız ve ivme maksimum değerlere ayarlandı");


  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto joints = move_group.getCurrentJointValues();
  if (joints.size() < 3)
  {
    RCLCPP_ERROR(node->get_logger(), "Joint state alınamadı!");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Current joints: %.3f %.3f %.3f",
              joints[0], joints[1], joints[2]);

  std::vector<double> target = joints;
  auto current = move_group.getCurrentJointValues();
  RCLCPP_INFO(node->get_logger(), "Şu anki pozisyon: %.3f", current[0]);

  
  // Başlangıç durumunu güncelle
  move_group.setStartStateToCurrentState();

  float q1, q2;
  calcula_joint_angles(0.2f, 0.2f, l1, l2, q1, q2);
  

  target[0] = q1;
  target[2] = q2;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 1 tamamlandı");


  std::vector<double> gripper_open = {0.4, 0.4, 0.4};
  gripper_group.setJointValueTarget(gripper_open);
  
  gripper_group.plan(gripper_plan);
  gripper_group.execute(gripper_plan);
  RCLCPP_INFO(node->get_logger(), "Gripper açıldı");

  
  target[1] = 0.18;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 2 tamamlandı");

  calcula_joint_angles(0.2f, 0.0f, l1, l2, q1, q2);
  RCLCPP_INFO(node->get_logger(), "IK Sonuçları: q1=%.3f, q2=%.3f", q1, q2);

  gripper_open = {-0.01, -0.01, -0.01};
  gripper_group.setJointValueTarget(gripper_open);
  gripper_group.plan(gripper_plan);
  gripper_group.execute(gripper_plan);
  RCLCPP_INFO(node->get_logger(), "Gripper kapandı");

  target[1] = 0.0;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 3 tamamlandı");

  calcula_joint_angles(0.4f, 0.0f, l1, l2, q1, q2);
  RCLCPP_INFO(node->get_logger(), "IK Sonuçları: q1=%.3f, q2=%.3f", q1, q2);
  target[0] = q1;
  target[2] = q2;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 4 tamamlandı");

  target[1] = 0.18;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 5 tamamlandı");

  gripper_open = {0.4, 0.4, 0.4};
  gripper_group.setJointValueTarget(gripper_open);
  gripper_group.plan(gripper_plan);
  gripper_group.execute(gripper_plan);
  RCLCPP_INFO(node->get_logger(), "Gripper açıldı");

  target[1] = 0.0;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 6 tamamlandı");

  gripper_open = {-0.01, -0.01, -0.01};
  gripper_group.setJointValueTarget(gripper_open);
  gripper_group.plan(gripper_plan);
  gripper_group.execute(gripper_plan);
  RCLCPP_INFO(node->get_logger(), "Gripper kapandı");

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.2;
  move_group.setPoseTarget(target_pose);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Pose hedefi tamamlandı");

  rclcpp::shutdown();
  return 0;
}
