#include "zyron_control/zyron_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <limits>
#include <thread>
#include <unordered_set>

namespace zyron_control
{
namespace
{

constexpr std::array<const char *, 10> kImuInterfaceNames{{
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}};

template<typename T, typename Parser>
T hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const T & default_value, Parser parser)
{
  const auto parameter = info.hardware_parameters.find(name);
  return parameter == info.hardware_parameters.end() ? default_value : parser(parameter->second);
}

}  // namespace

ZyronInterface::~ZyronInterface()
{
  stop_driver();
}

CallbackReturn ZyronInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  if (info_.joints.size() != 2 || info_.sensors.size() != 1 || info_.sensors[0].name != "imu")
  {
    RCLCPP_ERROR(
      get_logger(), "Zyron hardware requires two wheel joints and one sensor named 'imu'");
    return CallbackReturn::ERROR;
  }

  bool found_left = false;
  bool found_right = false;
  for (std::size_t index = 0; index < info_.joints.size(); ++index)
  {
    const auto & joint = info_.joints[index];
    if (joint.name == "wheel_left_joint")
    {
      left_joint_index_ = index;
      found_left = true;
    }
    else if (joint.name == "wheel_right_joint")
    {
      right_joint_index_ = index;
      found_right = true;
    }

    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' must expose one velocity command", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    std::unordered_set<std::string> state_names;
    for (const auto & state_interface : joint.state_interfaces)
    {
      state_names.insert(state_interface.name);
    }
    if (state_names.size() != 2 ||
      state_names.count(hardware_interface::HW_IF_POSITION) == 0 ||
      state_names.count(hardware_interface::HW_IF_VELOCITY) == 0)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose position and velocity state", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (!found_left || !found_right)
  {
    RCLCPP_ERROR(
      get_logger(), "Expected wheel_left_joint and wheel_right_joint in the hardware description");
    return CallbackReturn::ERROR;
  }

  std::unordered_set<std::string> imu_interface_names;
  for (const auto & state_interface : info_.sensors[0].state_interfaces)
  {
    imu_interface_names.insert(state_interface.name);
  }
  if (imu_interface_names.size() != kImuInterfaceNames.size())
  {
    RCLCPP_ERROR(get_logger(), "Sensor 'imu' must expose all ten standard IMU interfaces");
    return CallbackReturn::ERROR;
  }
  for (const char * interface_name : kImuInterfaceNames)
  {
    if (imu_interface_names.count(interface_name) == 0)
    {
      RCLCPP_ERROR(get_logger(), "Sensor 'imu' is missing interface '%s'", interface_name);
      return CallbackReturn::ERROR;
    }
  }

  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);
  velocity_commands_.assign(info_.joints.size(), 0.0);

  try
  {
    driver_config_.port = hardware_parameter<std::string>(
      info_, "port", driver_config_.port, [](const std::string & value) {return value;});
    driver_config_.baud_rate = hardware_parameter<int>(
      info_, "baud_rate", driver_config_.baud_rate,
      [](const std::string & value) {return std::stoi(value);});
    driver_config_.left_counts_per_rev = hardware_parameter<float>(
      info_, "left_counts_per_rev", driver_config_.left_counts_per_rev,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.right_counts_per_rev = hardware_parameter<float>(
      info_, "right_counts_per_rev", driver_config_.right_counts_per_rev,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.max_speed_rad_s = hardware_parameter<float>(
      info_, "max_speed_rad_s", driver_config_.max_speed_rad_s,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.start_threshold_rad_s = hardware_parameter<float>(
      info_, "start_threshold_rad_s", driver_config_.start_threshold_rad_s,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.stop_threshold_rad_s = hardware_parameter<float>(
      info_, "stop_threshold_rad_s", driver_config_.stop_threshold_rad_s,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.left_pwm_min = hardware_parameter<int>(
      info_, "left_pwm_min", driver_config_.left_pwm_min,
      [](const std::string & value) {return std::stoi(value);});
    driver_config_.right_pwm_min = hardware_parameter<int>(
      info_, "right_pwm_min", driver_config_.right_pwm_min,
      [](const std::string & value) {return std::stoi(value);});
    driver_config_.pwm_max = hardware_parameter<int>(
      info_, "pwm_max", driver_config_.pwm_max,
      [](const std::string & value) {return std::stoi(value);});
    driver_config_.left_kp = hardware_parameter<float>(
      info_, "left_kp", driver_config_.left_kp,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.left_ki = hardware_parameter<float>(
      info_, "left_ki", driver_config_.left_ki,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.right_kp = hardware_parameter<float>(
      info_, "right_kp", driver_config_.right_kp,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.right_ki = hardware_parameter<float>(
      info_, "right_ki", driver_config_.right_ki,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.max_acceleration_rad_s2 = hardware_parameter<float>(
      info_, "max_acceleration_rad_s2", driver_config_.max_acceleration_rad_s2,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.velocity_filter_tau_s = hardware_parameter<float>(
      info_, "velocity_filter_tau_s", driver_config_.velocity_filter_tau_s,
      [](const std::string & value) {return std::stof(value);});
    driver_config_.firmware_watchdog = std::chrono::milliseconds(hardware_parameter<int>(
      info_, "firmware_watchdog_ms", static_cast<int>(driver_config_.firmware_watchdog.count()),
      [](const std::string & value) {return std::stoi(value);}));
    driver_config_.startup_timeout = std::chrono::milliseconds(hardware_parameter<int>(
      info_, "startup_timeout_ms", static_cast<int>(driver_config_.startup_timeout.count()),
      [](const std::string & value) {return std::stoi(value);}));
    driver_config_.state_timeout = std::chrono::milliseconds(hardware_parameter<int>(
      info_, "state_timeout_ms", static_cast<int>(driver_config_.state_timeout.count()),
      [](const std::string & value) {return std::stoi(value);}));
    driver_config_.imu_timeout = std::chrono::milliseconds(hardware_parameter<int>(
      info_, "imu_timeout_ms", static_cast<int>(driver_config_.imu_timeout.count()),
      [](const std::string & value) {return std::stoi(value);}));
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(get_logger(), "Invalid Zyron hardware parameter: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  if (!ZyronSerialDriver::validate_config(driver_config_))
  {
    RCLCPP_ERROR(get_logger(), "Zyron serial driver configuration is invalid");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronInterface::on_configure(const rclcpp_lifecycle::State &)
{
  driver_ = std::make_unique<ZyronSerialDriver>(driver_config_);
  if (!start_driver_and_wait())
  {
    stop_driver();
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Zyron serial driver configured on %s", driver_config_.port.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronInterface::on_activate(const rclcpp_lifecycle::State &)
{
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  if (!driver_)
  {
    driver_ = std::make_unique<ZyronSerialDriver>(driver_config_);
  }
  if (!driver_->healthy())
  {
    if (!start_driver_and_wait())
    {
      stop_driver();
      return CallbackReturn::ERROR;
    }
  }
  else if (!wait_for_fresh_state())
  {
    stop_driver();
    return CallbackReturn::ERROR;
  }
  driver_->set_velocity_commands(0.0, 0.0);
  RCLCPP_INFO(get_logger(), "Zyron hardware interface active");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_driver();
  RCLCPP_INFO(get_logger(), "Zyron hardware interface deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_driver();
  driver_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronInterface::on_error(const rclcpp_lifecycle::State &)
{
  stop_driver();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZyronInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2 + kImuInterfaceNames.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index)
  {
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &position_states_[index]);
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[index]);
  }
  for (std::size_t index = 0; index < kImuInterfaceNames.size(); ++index)
  {
    state_interfaces.emplace_back("imu", kImuInterfaceNames[index], &imu_states_[index]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZyronInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index)
  {
    command_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_commands_[index]);
  }
  return command_interfaces;
}

hardware_interface::return_type ZyronInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_ || !driver_->healthy())
  {
    if (driver_)
    {
      driver_->request_stop();
    }
    return hardware_interface::return_type::ERROR;
  }

  DriverSample sample;
  if (!driver_->latest_sample(sample))
  {
    return hardware_interface::return_type::OK;
  }

  const bool wheel_fresh =
    ZyronSerialDriver::wheel_state_is_fresh(sample, driver_config_.state_timeout);
  const MotionInhibitReason reason = driver_->motion_inhibit_reason();
  if (reason != last_logged_inhibit_reason_)
  {
    if (reason == MotionInhibitReason::NONE)
    {
      RCLCPP_INFO(get_logger(), "Zyron motion gate rearmed");
    }
    else
    {
      RCLCPP_WARN(
        get_logger(), "Zyron motion inhibited: %s",
        ZyronSerialDriver::inhibit_reason_text(reason));
    }
    last_logged_inhibit_reason_ = reason;
  }
  if (sample.telemetry_sequence_gaps != last_telemetry_sequence_gaps_)
  {
    RCLCPP_WARN(
      get_logger(), "Zyron telemetry sequence gaps: %llu",
      static_cast<unsigned long long>(sample.telemetry_sequence_gaps));
    last_telemetry_sequence_gaps_ = sample.telemetry_sequence_gaps;
  }
  if (sample.host_crc_errors != last_host_crc_errors_ ||
    sample.host_frame_errors != last_host_frame_errors_)
  {
    RCLCPP_WARN(
      get_logger(), "Host receive errors: crc=%llu frame=%llu",
      static_cast<unsigned long long>(sample.host_crc_errors),
      static_cast<unsigned long long>(sample.host_frame_errors));
    last_host_crc_errors_ = sample.host_crc_errors;
    last_host_frame_errors_ = sample.host_frame_errors;
  }
  if (sample.firmware_crc_errors != last_firmware_crc_errors_ ||
    sample.firmware_frame_errors != last_firmware_frame_errors_)
  {
    RCLCPP_WARN(
      get_logger(), "ESP32 receive errors: crc=%u frame=%u",
      sample.firmware_crc_errors, sample.firmware_frame_errors);
    last_firmware_crc_errors_ = sample.firmware_crc_errors;
    last_firmware_frame_errors_ = sample.firmware_frame_errors;
  }
  if (sample.firmware_status != last_firmware_status_)
  {
    RCLCPP_INFO(
      get_logger(), "ESP32 status=0x%04x configured=%s imu_ready=%s command_stale=%s "
      "left_saturated=%s right_saturated=%s",
      sample.firmware_status,
      (sample.firmware_status & zyron_protocol::STATUS_CONFIGURED) != 0 ? "true" : "false",
      (sample.firmware_status & zyron_protocol::STATUS_IMU_READY) != 0 ? "true" : "false",
      (sample.firmware_status & zyron_protocol::STATUS_COMMAND_STALE) != 0 ? "true" : "false",
      (sample.firmware_status & zyron_protocol::STATUS_LEFT_SATURATED) != 0 ? "true" : "false",
      (sample.firmware_status & zyron_protocol::STATUS_RIGHT_SATURATED) != 0 ? "true" : "false");
    last_firmware_status_ = sample.firmware_status;
  }

  if (wheel_fresh)
  {
    position_states_[left_joint_index_] = sample.wheel_position[0];
    position_states_[right_joint_index_] = sample.wheel_position[1];
    velocity_states_[left_joint_index_] = sample.wheel_velocity[0];
    velocity_states_[right_joint_index_] = sample.wheel_velocity[1];
  }

  if (ZyronSerialDriver::imu_state_is_fresh(sample, driver_config_.imu_timeout))
  {
    std::copy(sample.imu_orientation.begin(), sample.imu_orientation.end(), imu_states_.begin());
    std::copy(
      sample.imu_angular_velocity.begin(), sample.imu_angular_velocity.end(),
      imu_states_.begin() + 4);
    std::copy(
      sample.imu_linear_acceleration.begin(), sample.imu_linear_acceleration.end(),
      imu_states_.begin() + 7);
  }
  else
  {
    std::fill(
      imu_states_.begin(), imu_states_.end(),
      std::numeric_limits<double>::quiet_NaN());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZyronInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_ || !driver_->healthy())
  {
    if (driver_)
    {
      driver_->request_stop();
    }
    return hardware_interface::return_type::ERROR;
  }

  driver_->set_velocity_commands(
    velocity_commands_[left_joint_index_], velocity_commands_[right_joint_index_]);
  return hardware_interface::return_type::OK;
}

bool ZyronInterface::start_driver_and_wait()
{
  if (!driver_)
  {
    RCLCPP_ERROR(get_logger(), "Zyron serial driver has not been created");
    return false;
  }
  if (!driver_->start())
  {
    RCLCPP_ERROR(
      get_logger(), "Failed to open Zyron serial port %s: %s",
      driver_config_.port.c_str(), driver_->startup_error().c_str());
    return false;
  }

  return wait_for_fresh_state();
}

bool ZyronInterface::wait_for_fresh_state()
{
  if (!driver_)
  {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + driver_config_.startup_timeout;
  DriverSample sample;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (!driver_->healthy())
    {
      RCLCPP_ERROR(get_logger(), "Zyron serial I/O stopped while waiting for telemetry");
      return false;
    }
    if (driver_->configured() && driver_->latest_sample(sample) &&
      sample.config_acknowledged && sample.motion_allowed &&
      ZyronSerialDriver::wheel_state_is_fresh(sample, driver_config_.state_timeout) &&
      ZyronSerialDriver::imu_state_is_fresh(sample, driver_config_.imu_timeout))
    {
      position_states_[left_joint_index_] = sample.wheel_position[0];
      position_states_[right_joint_index_] = sample.wheel_position[1];
      velocity_states_[left_joint_index_] = sample.wheel_velocity[0];
      velocity_states_[right_joint_index_] = sample.wheel_velocity[1];
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  driver_->latest_sample(sample);
  const auto now = std::chrono::steady_clock::now();
  const bool wheel_fresh =
    ZyronSerialDriver::wheel_state_is_fresh(sample, driver_config_.state_timeout, now);
  const bool imu_fresh =
    ZyronSerialDriver::imu_state_is_fresh(sample, driver_config_.imu_timeout, now);
  const auto age_ms = [now](
      bool valid, std::chrono::steady_clock::time_point stamp) -> long long {
      if (!valid || now < stamp)
      {
        return -1;
      }
      return std::chrono::duration_cast<std::chrono::milliseconds>(now - stamp).count();
    };
  RCLCPP_ERROR(
    get_logger(),
    "Timed out after %lld ms waiting for fresh telemetry on %s: "
    "configured=%s acknowledged=%s motion_allowed=%s inhibit='%s', "
    "wheel(valid=%s fresh=%s sequence=%llu age_ms=%lld), "
    "imu(valid=%s fresh=%s sequence=%llu age_ms=%lld), gaps=%llu status=0x%04x",
    static_cast<long long>(driver_config_.startup_timeout.count()),
    driver_config_.port.c_str(),
    driver_->configured() ? "true" : "false",
    sample.config_acknowledged ? "true" : "false",
    sample.motion_allowed ? "true" : "false",
    ZyronSerialDriver::inhibit_reason_text(driver_->motion_inhibit_reason()),
    sample.wheel_valid ? "true" : "false", wheel_fresh ? "true" : "false",
    static_cast<unsigned long long>(sample.wheel_sequence),
    age_ms(sample.wheel_valid, sample.wheel_stamp),
    sample.imu_valid ? "true" : "false", imu_fresh ? "true" : "false",
    static_cast<unsigned long long>(sample.imu_sequence),
    age_ms(sample.imu_valid, sample.imu_stamp),
    static_cast<unsigned long long>(sample.telemetry_sequence_gaps), sample.firmware_status);
  return false;
}

void ZyronInterface::stop_driver() noexcept
{
  if (driver_)
  {
    driver_->stop();
  }
}

}  // namespace zyron_control

PLUGINLIB_EXPORT_CLASS(zyron_control::ZyronInterface, hardware_interface::SystemInterface)
