#ifndef ZYRON_INTERFACE_HPP
#define ZYRON_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

namespace zyron_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ZyronInterface : public hardware_interface::SystemInterface
{
public:
  ZyronInterface();
  virtual ~ZyronInterface();

  // hardware_interface::SystemInterface (Jazzy API)
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // rclcpp_lifecycle::LifecycleNodeInterface
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Internal ROS2 node for topic I/O (hardware interfaces are not rclcpp nodes).
  // The node is added to the ControllerManager's executor when available (Jazzy+),
  // otherwise falls back to a private spin thread.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

  rclcpp::Executor::WeakPtr executor_;  // CM executor (from on_init params)
  bool node_added_to_executor_{false};

  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  // Cached state — updated by subscription callback, read by on_read()
  std::mutex state_mutex_;
  std::vector<double> position_states_;    // [left, right]
  std::vector<double> velocity_states_;    // [left, right]
  std::vector<double> velocity_commands_;  // [left, right]
};

}  // namespace zyron_control

#endif  // ZYRON_INTERFACE_HPP
