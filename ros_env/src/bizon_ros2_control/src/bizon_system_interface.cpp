#include "bizon_ros2_control/bizon_system_interface.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <limits>

// todo: add "serialComm.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bizon_ros2_control
{
hardware_interface::CallbackReturn BizonSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BizonSystemInterface"),
      "SystemInterface initialization failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "Parameters: ");
  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "\thw_start_sec_: %f", hw_start_sec_);
  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "\thw_stop_sec_: %f", hw_stop_sec_);
  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "\tjoint_size: %zu", info.joints.size());
  RCLCPP_INFO(
    rclcpp::get_logger(
      "BizonSystemInterface"), "\tros2_control_hardware_type: %s",
    info.hardware_parameters.at("ros2_control_hardware_type").c_str());

  hw_measured_positions_.resize(info.joints.size(), 0.0);
  hw_measured_velocities_.resize(info.joints.size(), 0.0);
  hw_position_commands_.resize(info.joints.size(), 0.0);
  hw_effort_commands_.resize(info.joints.size(), 0.0);
  joint_command_interface_types_.resize(info.joints.size());

  for (size_t i = 0; i < info.joints.size(); i++) {
    const hardware_interface::ComponentInfo & joint = info.joints[i];
    RCLCPP_INFO(
      rclcpp::get_logger(
        "BizonSystemInterface"), "Joint '%s' has the following command interfaces:",
      joint.name.c_str());
    for (const hardware_interface::InterfaceInfo & command_interface : joint.command_interfaces) {
      RCLCPP_INFO(
        rclcpp::get_logger("BizonSystemInterface"), "\t'%s'",
        command_interface.name.c_str());
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "BizonSystemInterface"), "Joint '%s' has the following state interfaces:",
      joint.name.c_str());
    for (const hardware_interface::InterfaceInfo & state_interface : joint.state_interfaces) {
      RCLCPP_INFO(
        rclcpp::get_logger("BizonSystemInterface"), "\t'%s'",
        state_interface.name.c_str());
    }

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "BizonSystemInterface"), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string & cmd_interface = joint.command_interfaces[0].name;
    if (cmd_interface != hardware_interface::HW_IF_POSITION &&
      cmd_interface != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "BizonSystemInterface"), "Joint '%s' has '%s' command interface found. 'position' or 'effort' expected.",
        joint.name.c_str(), cmd_interface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_command_interface_types_[i] = cmd_interface;
  }

  if (info_.hardware_parameters.at("ros2_control_hardware_type") == "rviz") {
    write_commands_ = &BizonSystemInterface::write_rviz_commands;
    read_sensors_ = &BizonSystemInterface::read_rviz_sensors;
  } else if (info_.hardware_parameters.at("ros2_control_hardware_type") == "isaac") {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name});

    node_ = rclcpp::Node::make_shared("topic_based_ros2_control_node", options);

    topic_based_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/" + info_.hardware_parameters["prefix"] + "/joint_commands", 10);

    topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/" + info_.hardware_parameters["prefix"] + "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
      {
        received_joint_states_msg_ptr_.set(std::move(joint_state_msg));
      });

    read_sensors_ = &BizonSystemInterface::read_isaac_sensors;
    write_commands_ = &BizonSystemInterface::write_isaac_commands;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "BizonSystemInterface"), "Unknown ros2_control_hardware_type: '%s'", info_.hardware_parameters.at(
        "ros2_control_hardware_type").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(
      "BizonSystemInterface"), "\tnamespace: %s", info_.hardware_parameters.at("prefix").c_str());
  RCLCPP_INFO(
    rclcpp::get_logger(
      "BizonSystemInterface"), "\tport_id: %s", info_.hardware_parameters.at("port_id").c_str());
  RCLCPP_INFO(
    rclcpp::get_logger(
      "BizonSystemInterface"), "\tros2_control_hardware_type: %s",
    info_.hardware_parameters.at("ros2_control_hardware_type").c_str());
  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "on_init() succeeded");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BizonSystemInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  RCLCPP_INFO(
    rclcpp::get_logger("BizonSystemInterface"), "Starting HW in %f sec...",
    hw_start_sec_);

  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "on_activate() succeeded");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BizonSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "on_deactivate() succeeded");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BizonSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_measured_positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_measured_velocities_[i]));
    std::string debug_msg = "export_state_interfaces exported position and velocity for joint '" +
      info_.joints[i].name + "'";
    RCLCPP_DEBUG(rclcpp::get_logger("bizon_system_interface"), debug_msg.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("bizon_system_interface"), "export_state_interfaces() succeeded");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BizonSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    const std::string & interface_type = joint_command_interface_types_[i];
    if (interface_type == hardware_interface::HW_IF_POSITION) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name,
          hardware_interface::HW_IF_POSITION,
          &hw_position_commands_[i]));
      RCLCPP_DEBUG(
        rclcpp::get_logger(
          "bizon_system_interface"), "export_command_interfaces exported position for joint '%s'",
        info_.joints[i].name.c_str());
    } else if (interface_type == hardware_interface::HW_IF_EFFORT) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name,
          hardware_interface::HW_IF_EFFORT,
          &hw_effort_commands_[i]));
      RCLCPP_DEBUG(
        rclcpp::get_logger(
          "bizon_system_interface"), "export_command_interfaces exported effort for joint '%s'",
        info_.joints[i].name.c_str());
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("bizon_system_interface"),
    "export_command_interfaces() succeeded");
  return command_interfaces;
}

hardware_interface::return_type BizonSystemInterface::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (this->*read_sensors_)();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BizonSystemInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (this->*write_commands_)();
  return hardware_interface::return_type::OK;
}

void BizonSystemInterface::read_rviz_sensors()
{
  RCLCPP_INFO_ONCE(
    rclcpp::get_logger(
      "bizon_system_interface"), "read_rviz_sensors() called from read_sensors");
  for (auto i = 0u; i < info_.joints.size(); i++) {
    if (joint_command_interface_types_[i] == hardware_interface::HW_IF_POSITION) {
      hw_measured_positions_[i] = hw_position_commands_[i];
    }
    // For effort-controlled joints in rviz mode, position remains unchanged
    hw_measured_velocities_[i] = 0.0;
  }
}

void BizonSystemInterface::write_rviz_commands()
{
  RCLCPP_INFO_ONCE(
    rclcpp::get_logger(
      "bizon_system_interface"), "write_rviz_commands() called from write_commands");
  for (auto i = 0u; i < info_.joints.size(); i++) {
  }
}

void BizonSystemInterface::read_isaac_sensors()
{
  RCLCPP_INFO_ONCE(rclcpp::get_logger("bizon_system_interface"), "read_isaac_sensors() called");

  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }

  std::shared_ptr<sensor_msgs::msg::JointState> joint_state_msg;
  received_joint_states_msg_ptr_.get(joint_state_msg);

  // NULL CHECK EKLEMELİSİN!
  if (!joint_state_msg) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("bizon_system_interface"),
      *node_->get_clock(),
      1000,           // 1 saniyede bir uyar
      "No joint_state message received yet from Isaac Sim");
    return;
  }

  for (size_t i = 0; i < joint_state_msg->name.size(); i++) {
    for (size_t j = 0; j < info_.joints.size(); j++) {
      if (joint_state_msg->name[i] == info_.joints[j].name) {
        hw_measured_positions_[j] = joint_state_msg->position[i];
        hw_measured_velocities_[j] = joint_state_msg->velocity.size() >
          i ? joint_state_msg->velocity[i] : 0.0;
      }
    }
  }
}

void BizonSystemInterface::write_isaac_commands()
{
  sensor_msgs::msg::JointState joint_state_msg;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    joint_state_msg.name.push_back(info_.joints[i].name);

    if (joint_command_interface_types_[i] == hardware_interface::HW_IF_POSITION) {
      joint_state_msg.position.push_back(hw_position_commands_[i]);
      joint_state_msg.effort.push_back(0.0);
    } else if (joint_command_interface_types_[i] == hardware_interface::HW_IF_EFFORT) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "bizon_system_interface"), "hw_effort_commands_[%zu]: %f, hw_position_commands_[%zu]: %f", i,
        hw_effort_commands_[i], i, hw_position_commands_[i]);
      joint_state_msg.position.push_back(hw_measured_positions_[i]);           // Keep current position
      joint_state_msg.effort.push_back(10.0);
    }
    joint_state_msg.velocity.push_back(0.0);
  }

  topic_based_joint_commands_publisher_->publish(joint_state_msg);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  bizon_ros2_control::BizonSystemInterface,
  hardware_interface::SystemInterface)
