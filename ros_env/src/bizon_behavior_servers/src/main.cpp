#include <memory>

#include "bizon_behavior_servers/behavior_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto recoveries_node = std::make_shared<behavior_server::BehaviorServer>();

  rclcpp::spin(recoveries_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}