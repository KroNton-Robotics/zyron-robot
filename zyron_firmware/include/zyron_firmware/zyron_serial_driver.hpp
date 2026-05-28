#ifndef ZYRON_SERIAL_DRIVER_HPP
#define ZYRON_SERIAL_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <libserial/SerialPort.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace zyron_firmware
{

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ZyronSerialDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ZyronSerialDriver(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~ZyronSerialDriver();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void read_from_mcu();
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Serial port
  LibSerial::SerialPort mcu_;
  std::string port_;

  // Position integration — accumulated from incoming velocity frames
  double pos_left_{0.0};
  double pos_right_{0.0};

  // Publishers / subscribers / timer
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr          imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  // Motor conversion parameters (declared as ROS params in on_configure)
  double max_speed_rad_s_{20.94};  // 200 RPM × 2π / 60
  double deadband_rad_s_{0.5};     // tune on your hardware
  int    pwm_min_{60};             // minimum PWM that actually moves the wheel
  int    pwm_max_{255};
  double reverse_scale_{1.0};   // >1.0 if backward is slower, <1.0 if faster
};

}  // namespace zyron_firmware

#endif  // ZYRON_SERIAL_DRIVER_HPP