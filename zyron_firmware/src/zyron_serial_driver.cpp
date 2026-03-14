#include "zyron_firmware/zyron_serial_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
// NOTE: do NOT include <libserial/SerialStreamBuf.h> — it re-declares
// LibSerial enums without their namespace and breaks BaudRate / ReadTimeout.
// SerialPort.h (pulled in by the header) provides everything we need.

namespace zyron_firmware
{

ZyronSerialDriver::ZyronSerialDriver(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("zyron_serial_driver", options)
{
}

ZyronSerialDriver::~ZyronSerialDriver()
{
  if (mcu_.IsOpen())
  {
    try
    {
      mcu_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(get_logger(),
                          "Something went wrong while closing serial port " << port_);
    }
  }
}

CallbackReturn ZyronSerialDriver::on_configure(const rclcpp_lifecycle::State &)
{
  port_            = declare_parameter<std::string>("port", "/dev/ttyUSB1");
  max_speed_rad_s_ = declare_parameter("max_speed_rad_s", 20.94);
  deadband_rad_s_  = declare_parameter("deadband_rad_s",  0.5);
  pwm_min_         = declare_parameter("pwm_min",         60);
  pwm_max_         = declare_parameter("pwm_max",         255);

  state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/zyron/joint_states", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/zyron/imu", 10);
  cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/zyron/wheel_commands", 10,
      std::bind(&ZyronSerialDriver::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured. Port: %s", port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronSerialDriver::on_activate(const rclcpp_lifecycle::State &)
{
  pos_left_ = pos_right_ = 0.0;

  try
  {
    mcu_.Open(port_);
    mcu_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    mcu_.FlushIOBuffers();
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(get_logger(), "Failed to open serial port " << port_);
    return CallbackReturn::FAILURE;
  }

  read_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ZyronSerialDriver::read_from_mcu, this));

  RCLCPP_INFO(get_logger(), "Serial driver active on %s", port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronSerialDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (read_timer_)
  {
    read_timer_->cancel();
    read_timer_.reset();
  }

  if (mcu_.IsOpen())
  {
    try
    {
      mcu_.Close();
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Error closing serial port " << port_);
    }
  }

  RCLCPP_INFO(get_logger(), "Serial driver deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronSerialDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  state_pub_.reset();
  imu_pub_.reset();
  cmd_sub_.reset();
  RCLCPP_INFO(get_logger(), "Serial driver cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZyronSerialDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (mcu_.IsOpen())
  {
    try
    {
      mcu_.Close();
    }
    catch (...) {}
  }
  return CallbackReturn::SUCCESS;
}

void ZyronSerialDriver::read_from_mcu()
{
  if (!mcu_.IsOpen())
    return;

  // Drain every complete line available this tick instead of reading
  // one line and leaving the rest to accumulate.
  while (mcu_.IsDataAvailable())
  {
    try
    {
      std::string line;
      mcu_.ReadLine(line, '\n', 10);

      if (line.empty())
        continue;

      // ── IMU frame  "I<yaw>,<pitch>,<roll>,<ax>,<ay>,<az>[,<gx>,<gy>,<gz>]"
      if (line.front() == 'I')
      {
        double yaw, pitch, roll, ax, ay, az;
        double gx = 0.0, gy = 0.0, gz = 0.0;

        if (std::sscanf(line.c_str(), "I%lf,%lf,%lf,%lf,%lf,%lf",
                        &yaw, &pitch, &roll, &ax, &ay, &az) != 6)
        {
          RCLCPP_WARN(get_logger(), "Malformed IMU line: %s", line.c_str());
          continue;
        }

        // Optional gyro — silently ignored if not present
        std::sscanf(line.c_str(), "I%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                    &yaw, &pitch, &roll, &ax, &ay, &az, &gx, &gy, &gz);

        // Euler → quaternion (ZYX convention)
        const double cy = std::cos(yaw * 0.5),   sy = std::sin(yaw * 0.5);
        const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        const double cr = std::cos(roll * 0.5),  sr = std::sin(roll * 0.5);

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;

        imu_pub_->publish(imu_msg);
        continue;
      }

      // ── Joint-state frame  "L<pos>,<vel>,R<pos>,<vel>"
      if (line.front() == 'L')
      {
        double pos_left, vel_left, pos_right, vel_right;
        if (std::sscanf(line.c_str(), "L%lf,%lf,R%lf,%lf",
                        &pos_left, &vel_left, &pos_right, &vel_right) != 4)
        {
          RCLCPP_WARN(get_logger(), "Malformed joint line: %s", line.c_str());
          continue;
        }

        pos_left_  = pos_left;
        pos_right_ = pos_right;

        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = now();
        js_msg.name         = {"wheel_left_joint", "wheel_right_joint"};
        js_msg.position     = {pos_left_,  pos_right_};
        js_msg.velocity     = {vel_left,   vel_right};
        state_pub_->publish(js_msg);
        continue;
      }

      RCLCPP_WARN(get_logger(), "Unknown serial frame: %s", line.c_str());
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // No complete line ready yet — come back next tick
      break;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Serial read error: %s", e.what());
      break;
    }
  }
}

void ZyronSerialDriver::command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2)
  {
    RCLCPP_WARN(get_logger(), "wheel_commands needs 2 values [left, right]");
    return;
  }

  if (!mcu_.IsOpen())
    return;

  auto radPerSecToPwm = [&](double vel) -> int {
    // 1. Deadband — below this threshold the motor stalls, output nothing
    if (std::abs(vel) < deadband_rad_s_)
      return 0;

    // 2. Preserve direction, work with magnitude
    const int    sign    = (vel > 0.0) ? 1 : -1;
    const double abs_vel = std::min(std::abs(vel), max_speed_rad_s_);

    // 3. Linear map: [deadband → max_speed] → [pwm_min → pwm_max]
    //    Jumps 0 → pwm_min cleanly, never commands the stall zone
    const double ratio = (abs_vel - deadband_rad_s_)
                       / (max_speed_rad_s_ - deadband_rad_s_);

    const int pwm = static_cast<int>(
        std::round(pwm_min_ + ratio * (pwm_max_ - pwm_min_)));

    return sign * pwm;
  };

  std::ostringstream ss;
  ss << "L" << radPerSecToPwm(msg->data[0])
     << ",R" << radPerSecToPwm(msg->data[1])
     << "\n";

  try
  {
    mcu_.Write(ss.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Failed to write to serial port: " << ss.str());
  }
}

} // namespace zyron_firmware