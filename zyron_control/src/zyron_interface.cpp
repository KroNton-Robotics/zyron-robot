#include "zyron_control/zyron_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <limits>
#include <thread>
#include <unordered_set>

namespace zyron_control
{


hardware_interface::CallbackReturn ZyronInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    port_ = info_.hardware_parameters["mcu_serial_port"];

    driver_ = std::make_shared<zyron_control::ZyronSerialDriver>(port_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZyronInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() !=0) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // for (const auto & [name, descr] : joint_command_interfaces_)
    // {
    //     RCLCPP_INFO(get_logger(), "COMMAND INTERFACE NAME: ");
    //     RCLCPP_INFO(get_logger(), name.c_str());
    // }
    // for (const auto & [name, descr] : joint_state_interfaces_)
    // {
    //     RCLCPP_INFO(get_logger(), "STATE INTERFACE NAME: ");
    //     RCLCPP_INFO(get_logger(), name.c_str());
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZyronInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);
    driver_->activateWithVelocityMode(left_motor_id_);
    driver_->activateWithVelocityMode(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZyronInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->deactivate(left_motor_id_);
    driver_->deactivate(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZyronInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    double left_vel = driver_->getVelocityRadianPerSec(left_motor_id_);
    double right_vel = -1.0 * driver_->getVelocityRadianPerSec(right_motor_id_);
    if (abs(left_vel) < 0.03) { left_vel = 0.0; }
    if (abs(right_vel) < 0.03) { right_vel = 0.0; }
    set_state("base_left_wheel_joint/velocity", left_vel);
    set_state("base_right_wheel_joint/velocity", right_vel);
    set_state("base_left_wheel_joint/position", get_state("base_left_wheel_joint/position") + left_vel * period.seconds());
    set_state("base_right_wheel_joint/position", get_state("base_right_wheel_joint/position") + right_vel * period.seconds());
    // RCLCPP_INFO(get_logger(), "left vel: %lf, right vel: %lf, left pos: %lf, right pos: %lf",
    //          left_vel, right_vel, get_state("base_left_wheel_joint/position"), get_state("base_right_wheel_joint/position"));
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZyronInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    driver_->setTargetVelocityRadianPerSec(left_motor_id_, get_command("base_left_wheel_joint/velocity"));
    driver_->setTargetVelocityRadianPerSec(right_motor_id_, -1.0 * get_command("base_right_wheel_joint/velocity"));
    // RCLCPP_INFO(get_logger(), "left vel: %lf, right vel: %lf", get_command("base_left_wheel_joint/velocity"), 
    //                     get_command("base_right_wheel_joint/velocity"));
    return hardware_interface::return_type::OK;
}

}  // namespace zyron_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zyron_control::ZyronInterface, hardware_interface::SystemInterface)
