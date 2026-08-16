#ifndef ZYRON_CONTROL__ZYRON_INTERFACE_HPP_
#define ZYRON_CONTROL__ZYRON_INTERFACE_HPP_

#include "zyron_control/zyron_serial_driver.hpp"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace zyron_control
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ZyronInterface : public hardware_interface::SystemInterface
{
public:
  ZyronInterface() = default;
  ~ZyronInterface() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool start_driver_and_wait();
  bool wait_for_fresh_state();
  void stop_driver() noexcept;

  DriverConfig driver_config_;
  std::unique_ptr<ZyronSerialDriver> driver_;

  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> velocity_commands_;
  std::array<double, 10> imu_states_{{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::size_t left_joint_index_{0};
  std::size_t right_joint_index_{1};
  MotionInhibitReason last_logged_inhibit_reason_{MotionInhibitReason::NONE};
  std::uint64_t last_telemetry_sequence_gaps_{0};
  std::uint64_t last_host_crc_errors_{0};
  std::uint64_t last_host_frame_errors_{0};
  std::uint16_t last_firmware_crc_errors_{0};
  std::uint16_t last_firmware_frame_errors_{0};
  std::uint16_t last_firmware_status_{0};
};

}  // namespace zyron_control

#endif  // ZYRON_CONTROL__ZYRON_INTERFACE_HPP_
