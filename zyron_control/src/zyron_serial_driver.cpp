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
  std::array<double, 8> ZyronSerialDriver::getParsedSerialMsg(const std::string &line)
  {
    std::array<double, 8> feedback_data = {0.0}; // Initialize all to 0.0
    std::stringstream ss(line);
    std::string chunk;

    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    double right_encoder_vel = 0.0, left_encoder_vel = 0.0;

    while (std::getline(ss, chunk, ','))
    {
      if (chunk.empty())
        continue;

      try
      {
        if (chunk.find("rp") == 0) right_encoder_vel = std::stod(chunk.substr(2));
        else if (chunk.find("rn") == 0) right_encoder_vel = -std::stod(chunk.substr(2));
        else if (chunk.find("lp") == 0) left_encoder_vel = std::stod(chunk.substr(2));
        else if (chunk.find("ln") == 0) left_encoder_vel = -std::stod(chunk.substr(2));
        else if (chunk.find("ax") == 0) ax = std::stod(chunk.substr(2));
        else if (chunk.find("ay") == 0) ay = std::stod(chunk.substr(2));
        else if (chunk.find("az") == 0) az = std::stod(chunk.substr(2));
        else if (chunk.find("y") == 0) yaw = std::stod(chunk.substr(1));
        else if (chunk.find("p") == 0) pitch = std::stod(chunk.substr(1));
        else if (chunk.find("r") == 0) roll = std::stod(chunk.substr(1)); // Safe because rp/rn are checked first
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
    if (!mcu_.IsOpen()) return "";

    // Take one bounded snapshot of the kernel buffer.  Do not loop on
    // IsDataAvailable(): the MCU can refill the buffer while it is being drained,
    // which used to keep ros2_control's real-time read() callback busy for tens of
    // milliseconds.
    constexpr int kMaxBytesPerCycle = 1024;
    const int available = mcu_.GetNumberOfBytesAvailable();
    if (available > 0)
    {
      std::string bytes;
      const auto bytes_to_read = static_cast<std::size_t>(
        std::min(available, kMaxBytesPerCycle));
      try
      {
        // A small, finite timeout protects against a race in which the number of
        // available bytes changes between the query and the read.
        mcu_.Read(bytes, bytes_to_read, 1);
      }
      catch (const LibSerial::ReadTimeout&)
      {
        // LibSerial preserves any bytes received before the timeout.
      }
      catch (const std::exception& e)
      {
        std::cout << "Serial read error: " << e.what() << std::endl;
        return "";
      }
      receive_buffer_.append(bytes);
    }

    // Return the newest complete frame and retain a partial trailing frame for
    // the next control cycle.
    const auto last_newline = receive_buffer_.rfind('\n');
    if (last_newline == std::string::npos)
    {
      // Prevent an unplugged/noisy device from growing this buffer forever.
      constexpr std::size_t kMaxPartialFrameSize = 4096;
      if (receive_buffer_.size() > kMaxPartialFrameSize)
      {
        receive_buffer_.erase(0, receive_buffer_.size() - kMaxPartialFrameSize);
      }
      return "";
    }

    const auto previous_newline =
      last_newline == 0 ? std::string::npos : receive_buffer_.rfind('\n', last_newline - 1);
    const auto frame_start =
      previous_newline == std::string::npos ? 0 : previous_newline + 1;
    std::string latest_response = receive_buffer_.substr(
      frame_start, last_newline - frame_start);
    receive_buffer_.erase(0, last_newline + 1);
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
