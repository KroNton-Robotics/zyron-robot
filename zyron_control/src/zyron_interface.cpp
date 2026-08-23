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


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZyronInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    // (Matches the exact URDF names from earlier)
    set_state("left_wheel_joint/velocity", 0.0);
    set_state("right_wheel_joint/velocity", 0.0);
    set_state("left_wheel_joint/position", 0.0);
    set_state("right_wheel_joint/position", 0.0);

    // IMU Initialization (Matches the exact URDF names from earlier)
    set_state("imu/orientation.x", 0.0);
    set_state("imu/orientation.y", 0.0);
    set_state("imu/orientation.z", 0.0);
    set_state("imu/orientation.w", 0.0);
    set_state("imu/angular_velocity.x", 0.0);
    set_state("imu/angular_velocity.y", 0.0);
    set_state("imu/angular_velocity.z", 0.0);
    set_state("imu/linear_acceleration.x", 0.0);
    set_state("imu/linear_acceleration.y", 0.0);
    set_state("imu/linear_acceleration.z", 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZyronInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZyronInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    std::string serial_msg=driver_->readSerialData();
    std::array<float, 8> parsed_serial_msg=driver_->getParsedSerialMsg(serial_msg);
    double left_vel = parsed_serial_msg[1];
    double right_vel = parsed_serial_msg[0];
    if (abs(left_vel) < 0.03) { left_vel = 0.0; }
    if (abs(right_vel) < 0.03) { right_vel = 0.0; }
    set_state("left_wheel_joint/velocity", left_vel);
    set_state("right_wheel_joint/velocity", right_vel);
    set_state("left_wheel_joint/position", get_state("left_wheel_joint/position") + left_vel * period.seconds());
    set_state("right_wheel_joint/position", get_state("right_wheel_joint/position") + right_vel * period.seconds());

    set_state("imu/orientation.x", parsed_serial_msg[2]);
    set_state("imu/orientation.y", parsed_serial_msg[3]);
    set_state("imu/orientation.z", parsed_serial_msg[4]);
    set_state("imu/orientation.w", parsed_serial_msg[5]);
    set_state("imu/linear_acceleration.x", parsed_serial_msg[6]);
    set_state("imu/linear_acceleration.y", parsed_serial_msg[7]);
    set_state("imu/linear_acceleration.z", parsed_serial_msg[8]);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZyronInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    
    driver_->buildRpsFrame(get_command("right_wheel_joint/velocity"),get_command("left_wheel_joint/velocity"));
    // RCLCPP_INFO(get_logger(), "left vel: %lf, right vel: %lf", get_command("left_wheel_joint/velocity"), 
    //                     get_command("right_wheel_joint/velocity"));
    return hardware_interface::return_type::OK;
}

}  // namespace zyron_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zyron_control::ZyronInterface, hardware_interface::SystemInterface)
