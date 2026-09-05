#include "zyron_control/zyron_serial_driver.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <array>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

namespace zyron_control
{

  ZyronSerialDriver::ZyronSerialDriver(std::string device_name) : port_(device_name), serial_fd_(-1)
  {
  }

  ZyronSerialDriver::~ZyronSerialDriver()
  {
    if (serial_port_.IsOpen())
    {
      serial_port_.Close();
    }
    serial_fd_ = -1;
  }

  int ZyronSerialDriver::init()
  {
    std::cout << "Initializing connection with robot." << std::endl;

    try
    {
      // Use libserial to open and configure the port
      serial_port_.Open(port_);
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    }
    catch (const LibSerial::OpenFailed &e)
    {
      std::cout << "Failed to open the port: " << port_ << " — " << e.what() << std::endl;
      return -1;
    }
    catch (const std::runtime_error &e)
    {
      std::cout << "Error configuring serial port: " << e.what() << std::endl;
      return -1;
    }

    // Get the underlying file descriptor for fast POSIX I/O
    serial_fd_ = serial_port_.GetFileDescriptor();

    // Set non-blocking mode on the fd
    int flags = fcntl(serial_fd_, F_GETFL, 0);
    fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK);

    // Ensure raw mode with non-blocking reads (VMIN=0, VTIME=0)
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) == 0)
    {
      tty.c_cflag |= (CREAD | CLOCAL);   // Enable receiver, ignore modem control lines
      tty.c_cflag &= ~CRTSCTS;           // Disable hardware flow control
      tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
      tty.c_cc[VMIN] = 0;                // Non-blocking: return immediately
      tty.c_cc[VTIME] = 0;               // No inter-byte timeout
      tcsetattr(serial_fd_, TCSANOW, &tty);
    }

    std::cout << "Succeeded to open the port!" << std::endl;
    return 0;
  }

  std::string ZyronSerialDriver::buildRpsFrame(int right_vel, int left_vel)
  {
    std::stringstream message_stream;
    char right_wheel_sign = right_vel >= 0 ? 'p' : 'n';
    char left_wheel_sign = left_vel >= 0 ? 'p' : 'n';

    std::string compensate_zeros_right = (std::abs(right_vel) < 10) ? "0" : "";
    std::string compensate_zeros_left = (std::abs(left_vel) < 10) ? "0" : "";

    message_stream << "r" << right_wheel_sign << compensate_zeros_right << std::abs(right_vel)
                   << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(left_vel) << ",\n";

    return message_stream.str();
  }

  std::array<double, 8> ZyronSerialDriver::getParsedSerialMsg(const std::string &line)
  {
    std::array<double, 8> feedback_data = {0.0};
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
        else if (chunk.find("r") == 0) roll = std::stod(chunk.substr(1));
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
    if (serial_fd_ < 0) return "";

    // Use raw POSIX read — non-blocking (O_NONBLOCK + VMIN=0)
    char buffer[1024];
    ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
    if (bytes_read > 0)
    {
      receive_buffer_.append(buffer, bytes_read);
    }

    const auto last_newline = receive_buffer_.rfind('\n');
    if (last_newline == std::string::npos)
    {
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
    if (serial_fd_ >= 0)
    {
      ssize_t bytes_written = ::write(serial_fd_, frame.c_str(), frame.size());
      if (bytes_written < 0)
      {
        std::cout << "Failed to write to serial port." << std::endl;
      }
    }
  }
} // namespace zyron_control