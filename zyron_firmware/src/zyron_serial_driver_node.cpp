#include "zyron_firmware/zyron_serial_driver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zyron_firmware::ZyronSerialDriver>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
