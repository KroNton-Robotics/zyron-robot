#ifndef ZYRON_CONTROL__ZYRON_SERIAL_DRIVER_HPP_
#define ZYRON_CONTROL__ZYRON_SERIAL_DRIVER_HPP_

#include <libserial/SerialPort.h>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

namespace zyron_control
{

  class ZyronSerialDriver
  {
  public:
    ZyronSerialDriver(std::string device_name);
    ~ZyronSerialDriver();
    int init();
    std::string buildRpsFrame(int right_vel, int left_vel);

    std::array<double, 8> getParsedSerialMsg(const std::string& line);
    std::string readSerialData();
    void sendSerialFrame(const std::string &frame);
    

  private:
    std::string port_;
    LibSerial::SerialPort mcu_;
  };

} // namespace zyron_control

#endif // ZYRON_CONTROL__ZYRON_SERIAL_DRIVER_HPP_
