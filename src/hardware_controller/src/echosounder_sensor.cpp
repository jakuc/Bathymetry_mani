#include "hardware_controller/echosounder_sensor.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

namespace
{
speed_t baud_to_speed(int baud)
{
  switch (baud)
  {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 115200: return B115200;
    default: return B4800;
  }
}
}  // namespace

hardware_interface::CallbackReturn EchosounderSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters.at("device_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  sensor_name_ = info_.sensors.at(0).name;

  state_range_ = std::numeric_limits<double>::quiet_NaN();
  state_water_temperature_ = std::numeric_limits<double>::quiet_NaN();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EchosounderSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(sensor_name_, "range", &state_range_);
  interfaces.emplace_back(sensor_name_, "water_temperature", &state_water_temperature_);
  return interfaces;
}

hardware_interface::CallbackReturn EchosounderSensor::on_configure(const rclcpp_lifecycle::State &)
{
  fd_ = ::open(device_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Nie można otworzyć portu %s", device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcgetattr nieudane na %s", device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const speed_t speed = baud_to_speed(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 bitów danych
  tty.c_cflag &= ~PARENB;                      // brak parzystości
  tty.c_cflag &= ~CSTOPB;                       // 1 bit stopu
  tty.c_cflag &= ~CRTSCTS;                      // brak sprzętowej kontroli przepływu
  tty.c_cflag |= CREAD | CLOCAL;

  cfmakeraw(&tty);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcsetattr nieudane na %s", device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EchosounderSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EchosounderSensor::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  poll_serial();
  return hardware_interface::return_type::OK;
}

void EchosounderSensor::poll_serial()
{
  char buf[256];
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0)
  {
    line_buffer_.append(buf, static_cast<size_t>(n));
  }

  size_t newline;
  while ((newline = line_buffer_.find('\n')) != std::string::npos)
  {
    std::string line = line_buffer_.substr(0, newline);
    line_buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    process_line(line);
  }
}

void EchosounderSensor::process_line(const std::string & line)
{
  const size_t star = line.find('*');
  const std::string body = (star == std::string::npos) ? line : line.substr(0, star);

  std::vector<std::string> fields;
  size_t start = 0;
  while (true)
  {
    const size_t comma = body.find(',', start);
    fields.push_back(body.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    if (comma == std::string::npos)
    {
      break;
    }
    start = comma + 1;
  }

  if (fields.empty())
  {
    return;
  }

  const std::string & tag = fields[0];
  const bool is_dpt = tag.size() >= 3 && tag.compare(tag.size() - 3, 3, "DPT") == 0;
  const bool is_mtw = tag.size() >= 3 && tag.compare(tag.size() - 3, 3, "MTW") == 0;

  try
  {
    if (is_dpt && fields.size() > 1)
    {
      state_range_ = fields[1].empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(fields[1]);
    }
    else if (is_mtw && fields.size() > 1 && !fields[1].empty())
    {
      state_water_temperature_ = std::stod(fields[1]);
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger_, "Nie udało się sparsować zdania NMEA '%s': %s", line.c_str(), e.what());
  }
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::EchosounderSensor, hardware_interface::SensorInterface)
