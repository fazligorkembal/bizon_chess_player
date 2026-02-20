#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_param_node");
    RCLCPP_INFO(node->get_logger(), "Node started");
    RCLCPP_INFO(node->get_logger(), "Namespace: %s", node->get_namespace());
    // Declare parameters with defaults
    node->declare_parameter<std::string>("robot_name", "unknown");
    node->declare_parameter<double>("speed", 0.0);
    node->declare_parameter<double>("max_acceleration", 0.0);
    node->declare_parameter<bool>("enabled", false);
    node->declare_parameter("joints", std::vector<std::string>({"default"}));
    node->declare_parameter("home_position", std::vector<double>({0.0}));
    node->declare_parameter<double>("timeout", 1.0);
    node->declare_parameter<int>("retry_count", 1);
    node->declare_parameter<bool>("debug_mode", false);
    node->declare_parameter<double>("acceleration");

    RCLCPP_INFO(node->get_logger(), "Parameters:");
    RCLCPP_INFO(node->get_logger(), "  robot_name: %s", node->get_parameter("robot_name").as_string().c_str());
    RCLCPP_INFO(node->get_logger(), "  speed: %.2f", node->get_parameter("speed").as_double());
    RCLCPP_INFO(node->get_logger(), "  max_acceleration: %.2f", node->get_parameter("max_acceleration").as_double());
    RCLCPP_INFO(node->get_logger(), "  enabled: %s", node->get_parameter("enabled").as_bool() ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "  acceleration: %.2f", node->get_parameter("acceleration").as_double());

    for (const auto& param : node->list_parameters({}, 10).names) {
        RCLCPP_INFO(node->get_logger(), "  [all] %s", param.c_str());
    }

    

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
