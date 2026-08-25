#ifndef ZYRON_CONTROL__ZYRON_INTERFACE_HPP_
#define ZYRON_CONTROL__ZYRON_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "zyron_control/zyron_serial_driver.hpp"

namespace zyron_control {

class ZyronInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<zyron_control::ZyronSerialDriver> driver_;
    std::string port_;
    

}; // class ZyronInterface

} // namespace zyron_control


#endif