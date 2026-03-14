#include "zyron_control/zyron_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <chrono>

namespace zyron_control
{

ZyronInterface::ZyronInterface()
{
}

ZyronInterface::~ZyronInterface()
{
  if (running_)
  {
    running_ = false;
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }
}

CallbackReturn ZyronInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  const size_t n = info_.joints.size();
  position_states_.assign(n, 0.0);
  velocity_states_.assign(n, 0.0);
  velocity_commands_.assign(n, 0.0);

  // Store the CM executor for use in on_activate
  executor_ = params.executor;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZyronInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZyronInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn ZyronInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ZyronInterface"), "Activating hardware interface ...");

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::fill(position_states_.begin(),   position_states_.end(),   0.0);
    std::fill(velocity_states_.begin(),   velocity_states_.end(),   0.0);
    std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  }

  node_ = std::make_shared<rclcpp::Node>("zyron_hw_interface");

  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/zyron/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&ZyronInterface::joint_state_callback, this, std::placeholders::_1));

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/zyron/wheel_commands", 10);

  // Prefer adding node to the CM executor (zero extra threads)
  auto executor = executor_.lock();
  if (executor)
  {
    executor->add_node(node_);
    node_added_to_executor_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ZyronInterface"),
      "Internal node added to ControllerManager executor");
  }
  else
  {
    // Fallback: own spin thread
    running_ = true;
    spin_thread_ = std::thread([this]() {
      while (running_)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    RCLCPP_INFO(rclcpp::get_logger("ZyronInterface"),
      "Internal node spinning on dedicated thread");
  }

  RCLCPP_INFO(rclcpp::get_logger("ZyronInterface"), "Hardware interface active");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ZyronInterface"), "Deactivating hardware interface ...");

  if (node_added_to_executor_)
  {
    auto executor = executor_.lock();
    if (executor)
    {
      executor->remove_node(node_);
    }
    node_added_to_executor_ = false;
  }
  else
  {
    running_ = false;
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }

  state_sub_.reset();
  cmd_pub_.reset();
  node_.reset();

  RCLCPP_INFO(rclcpp::get_logger("ZyronInterface"), "Hardware interface stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZyronInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // States are kept up-to-date by joint_state_callback; nothing extra needed.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZyronInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!cmd_pub_)
  {
    return hardware_interface::return_type::OK;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = velocity_commands_;  // [left, right]

  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

void ZyronInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    for (size_t j = 0; j < msg->name.size(); j++)
    {
      if (msg->name[j] == info_.joints[i].name)
      {
        if (j < msg->position.size()) { position_states_[i] = msg->position[j]; }
        if (j < msg->velocity.size()) { velocity_states_[i] = msg->velocity[j]; }
        break;
      }
    }
  }
}

}  // namespace zyron_control

PLUGINLIB_EXPORT_CLASS(zyron_control::ZyronInterface, hardware_interface::SystemInterface)
