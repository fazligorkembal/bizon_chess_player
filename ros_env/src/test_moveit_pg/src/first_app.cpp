#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

inline bool calcula_joint_angles(
    float x, float y,
    float l1, float l2,
    float &q1, float &q2)
{
    float r2 = x*x + y*y;

    float D = (r2 - l1*l1 - l2*l2) / (2.0f * l1 * l2);

    // ---- KRİTİK SATIR ----
    D = std::clamp(D, -1.0f, 1.0f);

    float inside = 1.0f - D*D;

    // numerical safety
    if (inside < 0.0f)
        inside = 0.0f;

    float s = std::sqrt(inside);

    q2 = std::atan2(s, D);
    q1 = std::atan2(y, x) - std::atan2(l2 * std::sin(q2),
                                       l1 + l2 * std::cos(q2));

    return true;
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
      "hand_group",
      "robot_description",
      node->get_namespace()
  );

  moveit::planning_interface::MoveGroupInterface move_group(node, options);
  moveit::planning_interface::MoveGroupInterface hand_group(node, options_eef);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  // Hız ve ivme ayarları
  move_group.setMaxVelocityScalingFactor(1.0);  // Maksimum hız
  move_group.setMaxAccelerationScalingFactor(1.0);  // Maksimum ivme
  hand_group.setMaxVelocityScalingFactor(1.0);
  hand_group.setMaxAccelerationScalingFactor(1.0);

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
  /*
  calcula_joint_angles(0.2f, 0.2f, l1, l2, q1, q2);
  target[0] = q1;
  target[2] = q2;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 1 tamamlandı");
  */


  calcula_joint_angles(-0.0185928572f + 0.3f, -0.0185943f, l1, l2, q1, q2);
  RCLCPP_INFO(node->get_logger(), "Hedef eklem açıları: q1=%.3f, q2=%.3f", q1, q2);
  target[0] = q1;
  target[2] = q2;
  move_group.setJointValueTarget(target);
  move_group.plan(plan);
  move_group.execute(plan);
  RCLCPP_INFO(node->get_logger(), "Execute 4 tamamlandı");

  rclcpp::shutdown();
  return 0;
}
