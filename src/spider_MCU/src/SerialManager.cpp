#include "spider_MCU/SerialManager.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace MCU
{
namespace
{

speed_t to_termios_baudrate(uint32_t baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return 0;
  }
}

}  // namespace

SerialManager::SerialManager() = default;

SerialManager::~SerialManager()
{
  disconnect();
}

bool SerialManager::connect(const std::string & port, uint32_t baudrate)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (connected_) {
    close_port_locked();
  }

  return open_port_locked(port, baudrate);
}

void SerialManager::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  close_port_locked();
}

bool SerialManager::is_connected() const
{
  return connected_.load();
}

std::string SerialManager::port() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return port_;
}

uint32_t SerialManager::baudrate() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return baudrate_;
}

bool SerialManager::write_line(const std::string & line)
{
  return write_bytes(line + "\n");
}

bool SerialManager::write_bytes(const std::string & bytes)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!connected_ || fd_ < 0) {
    return false;
  }

  std::size_t total_written = 0;
  while (total_written < bytes.size()) {
    const ssize_t written = ::write(
      fd_,
      bytes.data() + total_written,
      bytes.size() - total_written);

    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      connected_ = false;
      return false;
    }

    total_written += static_cast<std::size_t>(written);
  }

  return true;
}

std::optional<std::string> SerialManager::read_line(std::chrono::milliseconds timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!connected_ || fd_ < 0) {
    return std::nullopt;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto newline_pos = rx_buffer_.find('\n');
    if (newline_pos != std::string::npos) {
      std::string line = rx_buffer_.substr(0, newline_pos);
      rx_buffer_.erase(0, newline_pos + 1);

      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      return line;
    }

    char buffer[256];
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));

    if (n > 0) {
      rx_buffer_.append(buffer, static_cast<std::size_t>(n));
      continue;
    }

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        ::usleep(1000);
        continue;
      }
      if (errno == EINTR) {
        continue;
      }

      connected_ = false;
      return std::nullopt;
    }

    // n == 0
    ::usleep(1000);
  }

  return std::nullopt;
}

std::size_t SerialManager::available() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!connected_ || fd_ < 0) {
    return 0;
  }

  int bytes_available = 0;
  if (::ioctl(fd_, FIONREAD, &bytes_available) < 0) {
    return 0;
  }

  if (bytes_available < 0) {
    return 0;
  }

  return static_cast<std::size_t>(bytes_available);
}

void SerialManager::flush_input()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!connected_ || fd_ < 0) {
    return;
  }

  ::tcflush(fd_, TCIFLUSH);
  rx_buffer_.clear();
}

void SerialManager::flush_output()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!connected_ || fd_ < 0) {
    return;
  }

  ::tcflush(fd_, TCOFLUSH);
}

bool SerialManager::open_port_locked(const std::string & port, uint32_t baudrate)
{
  const speed_t speed = to_termios_baudrate(baudrate);
  if (speed == 0) {
    return false;
  }

  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }

  termios tty {};
  if (::tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return false;
  }

  ::cfmakeraw(&tty);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    ::close(fd);
    return false;
  }

  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return false;
  }

  fd_ = fd;
  port_ = port;
  baudrate_ = baudrate;
  rx_buffer_.clear();
  connected_ = true;

  return true;
}

void SerialManager::close_port_locked()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  connected_ = false;
  rx_buffer_.clear();
  port_.clear();
}

}  // namespace MCU