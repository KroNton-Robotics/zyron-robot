#include "zyron_control/zyron_serial_driver.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <array>

namespace zyron_control
{

  ZyronSerialDriver::ZyronSerialDriver(std::string device_name) : port_(device_name)
  {
  }

  ZyronSerialDriver::~ZyronSerialDriver()
  {
    if (mcu_.IsOpen())
    {
      mcu_.Close();
    }
  }

  int ZyronSerialDriver::init()
  {
    std::cout << "Initializing connection with robot." << std::endl;
    try
    {
      mcu_.Open(port_);
      mcu_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      std::cout << "Succeeded to open the port!" << std::endl;
      return 0; // Added return
    }
    catch (const LibSerial::OpenFailed &)
    {
      std::cout << "Failed to open the port!" << std::endl;
      return -1; // Added return
    }
  }

  std::string ZyronSerialDriver::buildRpsFrame(int right_vel, int left_vel)
  {
    std::stringstream message_stream;
    char right_wheel_sign = right_vel >= 0 ? 'p' : 'n';
    char left_wheel_sign = left_vel >= 0 ? 'p' : 'n';

    std::string compensate_zeros_right = (std::abs(right_vel) < 10) ? "0" : "";
    std::string compensate_zeros_left = (std::abs(left_vel) < 10) ? "0" : "";

    // Removed std::fixed and setprecision(2) since we are dealing with integers
    message_stream << "r" << right_wheel_sign << compensate_zeros_right << std::abs(right_vel)
                   << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(left_vel) << ",\n"; // Added \n just in case MCU needs it

    return message_stream.str();
  }

  // Changed to std::array to avoid dynamic memory allocation in the RT loop
  std::array<float, 8> ZyronSerialDriver::getParsedSerialMsg(const std::string &line)
  {
    std::array<float, 8> feedback_data = {0.0f}; // Initialize all to 0.0
    std::stringstream ss(line);
    std::string chunk;

    float yaw = 0.0, pitch = 0.0, roll = 0.0;
    float ax = 0.0, ay = 0.0, az = 0.0;
    float right_encoder_vel = 0.0, left_encoder_vel = 0.0;

    while (std::getline(ss, chunk, ','))
    {
      if (chunk.empty())
        continue;

      try
      {
        if (chunk.find("rp") == 0) right_encoder_vel = std::stof(chunk.substr(2));
        else if (chunk.find("rn") == 0) right_encoder_vel = -std::stof(chunk.substr(2));
        else if (chunk.find("lp") == 0) left_encoder_vel = std::stof(chunk.substr(2));
        else if (chunk.find("ln") == 0) left_encoder_vel = -std::stof(chunk.substr(2));
        else if (chunk.find("ax") == 0) ax = std::stof(chunk.substr(2));
        else if (chunk.find("ay") == 0) ay = std::stof(chunk.substr(2));
        else if (chunk.find("az") == 0) az = std::stof(chunk.substr(2));
        else if (chunk.find("y") == 0) yaw = std::stof(chunk.substr(1));
        else if (chunk.find("p") == 0) pitch = std::stof(chunk.substr(1));
        else if (chunk.find("r") == 0) roll = std::stof(chunk.substr(1)); // Safe because rp/rn are checked first
      }
      catch (const std::exception &e)
      {
        // Silently ignore malformed chunks (standard for noisy serial)
      }
    }

    feedback_data[0] = right_encoder_vel;
    feedback_data[1] = left_encoder_vel;
    feedback_data[2] = roll;
    feedback_data[3] = pitch;
    feedback_data[4] = yaw;
    feedback_data[5] = ax;
    feedback_data[6] = ay;
    feedback_data[7] = az;

    return feedback_data;
  }

  std::string ZyronSerialDriver::readSerialData()
  {
    if (!mcu_.IsOpen()) return ""; // Fixed return

    std::string latest_response = "";

    // Read ALL available lines to clear the buffer, but only keep the newest one
    while (mcu_.IsDataAvailable()) 
    {
      std::string response;
      try {
        mcu_.ReadLine(response, '\n', 5);
        if (!response.empty()) {
           latest_response = response; 
        }
      } 
      catch (const LibSerial::ReadTimeout&) {
        break; // Timeout, stop reading
      }
      catch (const std::exception& e) {
        std::cout << "Serial read error: " << e.what() << std::endl;
        break;
      }
    }
    
    // Will return empty string if no valid data was found, otherwise the newest line
    return latest_response; 
  }


  void ZyronSerialDriver::sendSerialFrame(const std::string &frame)
  {
    try
    {
      if (mcu_.IsOpen())
      {
        mcu_.Write(frame);
      }
    }
    catch (const std::exception& e) // Catch standard exceptions rather than (...)
    {
      std::cout << "Failed to send message to port: " << e.what() << std::endl;
    }
  }
} // namespace zyron_control