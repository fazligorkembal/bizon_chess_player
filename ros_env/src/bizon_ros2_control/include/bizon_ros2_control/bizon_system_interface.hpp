#ifndef BIZON_ROS2_CONTROL__BIZON_SYSTEM_INTERFACE_HPP_
#define BIZON_ROS2_CONTROL__BIZON_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// todo: add serialComm.h
#include "rclcpp/macros.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace bizon_ros2_control
{
    class BizonSystemInterface : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // todo: add serialComm uart_comm;
    protected:
        void nan_command_interfaces_();
        void nan_state_interfaces_();
        bool verify_number_of_joints_();
        bool verify_joint_command_interfaces_();
        bool verify_joint_state_interfaces_();
        bool verify_sensors_();
        // bool verify_auxiliary_sensor_();
        // bool verify_estimated_ft_sensor_();

        // void nan_last_hw_states_();
        // void update_last_hw_states_();

        int radian_angle_to_step_converter(double angle_rad, int micro_steps, double step_angles_deg, double gear_ratio);
        int distance_to_step_converter(double distance_m, int micro_steps, double step_angles_deg, double gear_ratio);

        std::vector<double> hw_position_commands_;
        std::vector<double> hw_effort_commands_;
        std::vector<double> hw_measured_positions_;
        std::vector<double> hw_measured_velocities_;

        std::vector<std::string> joint_command_interface_types_;

        void (BizonSystemInterface::*write_commands_)();
        void write_isaac_commands();
        void write_rviz_commands();

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_joint_commands_publisher_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_states_subscriber_;
        realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::JointState>> received_joint_states_msg_ptr_{nullptr};
        
        void (BizonSystemInterface::*read_sensors_)();
        void read_isaac_sensors();
        void read_rviz_sensors();
        float x = -4.0;

        std::queue<sensor_msgs::msg::JointState> previous_joint_states_; // last 2 commands
        double hw_start_sec_ = 3.0; // todo: parameterize these values
        double hw_stop_sec_ = 1.0; //  todo: parameterize these values

        rclcpp::Node::SharedPtr node_;
        sensor_msgs::msg::JointState latest_joint_state_;
    };

}

#endif // BIZON_ROS2_CONTROL__BIZON_SYSTEM_INTERFACE_HPP_