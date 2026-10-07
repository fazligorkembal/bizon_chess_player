#include "rclcpp/rclcpp.hpp"

#include "bizon_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_DEBUG(
    rclcpp::get_logger(
      "bizon_lifecycle_manager_main"), "Bizon Lifecycle Manager Node Started");
  RCLCPP_INFO(
    rclcpp::get_logger(
      "bizon_lifecycle_manager_main"), "Bizon Lifecycle Manager Node Running");

  auto lifecycle_manager_node = std::make_shared<bizon_lifecycle_manager::LifecycleManager>();
  rclcpp::spin(lifecycle_manager_node);

  rclcpp::shutdown();

  return 0;
}
