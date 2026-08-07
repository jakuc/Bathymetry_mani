#include "hardware_controller/imu_sensor.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
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
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B9600;
  }
}

int16_t be16(const uint8_t * p)
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

hardware_interface::CallbackReturn ImuSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.empty())
  {
    RCLCPP_ERROR(logger_, "Brak zadeklarowanego <sensor> w bloku ros2_control");
    return hardware_interface::CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors[0].name;

  const auto param = [this](const std::string & key, const std::string & fallback) {
    const auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? it->second : fallback;
  };

  device_port_ = param("device_port", "/dev/serial0");
  baud_rate_ = std::stoi(param("baud_rate", "9600"));
  configure_module_ = param("configure_module", "true") == "true";
  quat_wxyz_ = param("quaternion_order", "wxyz") == "wxyz";
  // stoi z bazą 0 rozumie zarówno "0xB5", jak i "181".
  module_cmd_ = static_cast<uint8_t>(std::stoi(param("module_cmd", "0xB5"), nullptr, 0));

  // Wszystko, czego moduł nie przyśle, ma zostać NaN-em - patrz komentarz w hpp.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto & v : orientation_) { v = nan; }
  for (auto & v : angular_velocity_) { v = nan; }
  for (auto & v : linear_acceleration_) { v = nan; }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ImuSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  // Nazwy narzucone przez imu_sensor_broadcaster z ros2_controllers - dzięki
  // nim nie musimy pisać własnego broadcastera, tylko spinamy gotowy.
  interfaces.emplace_back(sensor_name_, "orientation.x", &orientation_[0]);
  interfaces.emplace_back(sensor_name_, "orientation.y", &orientation_[1]);
  interfaces.emplace_back(sensor_name_, "orientation.z", &orientation_[2]);
  interfaces.emplace_back(sensor_name_, "orientation.w", &orientation_[3]);
  interfaces.emplace_back(sensor_name_, "angular_velocity.x", &angular_velocity_[0]);
  interfaces.emplace_back(sensor_name_, "angular_velocity.y", &angular_velocity_[1]);
  interfaces.emplace_back(sensor_name_, "angular_velocity.z", &angular_velocity_[2]);
  interfaces.emplace_back(sensor_name_, "linear_acceleration.x", &linear_acceleration_[0]);
  interfaces.emplace_back(sensor_name_, "linear_acceleration.y", &linear_acceleration_[1]);
  interfaces.emplace_back(sensor_name_, "linear_acceleration.z", &linear_acceleration_[2]);
  // Poza standardem: status kalibracji. imu_sensor_broadcaster tego nie czyta,
  // ale bez tego nie da się stwierdzić, czy fuzja jest już wiarygodna.
  interfaces.emplace_back(sensor_name_, "calib_sys", &calib_sys_);
  interfaces.emplace_back(sensor_name_, "calib_gyr", &calib_gyr_);
  interfaces.emplace_back(sensor_name_, "calib_acc", &calib_acc_);
  interfaces.emplace_back(sensor_name_, "calib_mag", &calib_mag_);
  return interfaces;
}

hardware_interface::CallbackReturn ImuSensor::on_configure(const rclcpp_lifecycle::State &)
{
  fd_ = ::open(device_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(
      logger_, "Nie można otworzyć portu %s (%s). Na Pi ttyS0 potrafi trzymać "
      "serial-getty@ttyS0 — sprawdź, czy jest wyłączony.",
      device_port_.c_str(), std::strerror(errno));
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
  tty.c_cflag &= ~CSTOPB;                      // 1 bit stopu
  tty.c_cflag &= ~CRTSCTS;                     // brak sprzętowej kontroli przepływu
  tty.c_cflag |= CREAD | CLOCAL;

  cfmakeraw(&tty);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcsetattr nieudane na %s", device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (configure_module_)
  {
    send_config();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ImuSensor::send_config()
{
  // Konfiguracja modułu: 0xAA + cmd + suma. Bity cmd:
  //   7 AUTO (sam nadaje), 6 100 Hz, 5 50 Hz, 4 Q4, 3 EULER, 2 GYRO, 1 MAG, 0 ACC
  //
  // Domyślne 0xB5 = AUTO + 50 Hz + Q4 + GYRO + ACC, czyli dokładnie to, czego
  // potrzebuje sensor_msgs/Imu. Świadomie bierzemy KWATERNION, nie Eulera:
  // kanały Eulera tego modułu nazywają się inaczej niż w ROS (jego "ROLL" to
  // obrót wokół Y i jest przycięty do +/-90, a "PITCH" chodzi pełne +/-180 —
  // w REP-103 jest odwrotnie), więc każde przejście przez Eulera to okazja do
  // pomyłki znaku. Kwaternion jest wolny od konwencji nazewniczej.
  //
  // Zapis jest TRWAŁY - moduł pamięta ustawienie po odłączeniu zasilania.
  //
  // Uwaga na przepustowość: przy 9600 8N1 (960 B/s) ramka Q4+GYR+ACC ma 26 B,
  // czyli maksymalnie ~37 Hz — mniej niż żądane 50 Hz. Na pełne 50 Hz trzeba
  // podnieść baud modułu do 115200.
  const uint8_t sum = static_cast<uint8_t>(ImuProtocol::kConfigPrefix + module_cmd_);
  const uint8_t cmd[3] = {ImuProtocol::kConfigPrefix, module_cmd_, sum};
  const ssize_t n = ::write(fd_, cmd, sizeof(cmd));
  if (n != static_cast<ssize_t>(sizeof(cmd)))
  {
    RCLCPP_WARN(logger_, "Nie udało się wysłać konfiguracji do modułu IMU");
    return;
  }
  RCLCPP_INFO(
    logger_, "IMU skonfigurowane: cmd=0x%02X (suma 0x%02X) na %s @ %d",
    module_cmd_, sum, device_port_.c_str(), baud_rate_);
}

hardware_interface::CallbackReturn ImuSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

int ImuSensor::expected_data_size(uint8_t mask) const
{
  int n = 0;
  if (mask & ImuProtocol::kMaskAcc) { n += ImuProtocol::kSizeAcc; }
  if (mask & ImuProtocol::kMaskMag) { n += ImuProtocol::kSizeMag; }
  if (mask & ImuProtocol::kMaskGyr) { n += ImuProtocol::kSizeGyr; }
  if (mask & ImuProtocol::kMaskEuler) { n += ImuProtocol::kSizeEuler; }
  if (mask & ImuProtocol::kMaskQuat) { n += ImuProtocol::kSizeQuat; }
  return n;
}

void ImuSensor::decode(uint8_t mask, const uint8_t * data, uint8_t calib)
{
  // Kolejność bloków w ramce jest stała: ACC, MAG, GYR, EULER(YRP), Q4 —
  // obecne są tylko te, których bit stoi w masce.
  const uint8_t * p = data;

  if (mask & ImuProtocol::kMaskAcc)
  {
    for (int i = 0; i < 3; ++i)
    {
      linear_acceleration_[i] = be16(p + 2 * i) / ImuProtocol::kAccPerMs2;
    }
    p += ImuProtocol::kSizeAcc;
  }
  if (mask & ImuProtocol::kMaskMag)
  {
    p += ImuProtocol::kSizeMag;  // magnetometru nie wystawiamy
  }
  if (mask & ImuProtocol::kMaskGyr)
  {
    for (int i = 0; i < 3; ++i)
    {
      // moduł podaje deg/s, sensor_msgs/Imu chce rad/s
      angular_velocity_[i] = (be16(p + 2 * i) / ImuProtocol::kGyrPerDps) * M_PI / 180.0;
    }
    p += ImuProtocol::kSizeGyr;
  }
  if (mask & ImuProtocol::kMaskEuler)
  {
    // Eulera świadomie NIE mapujemy na orientację: nazwy kanałów tego modułu są
    // zamienione względem ROS (patrz komentarz przy send_config), a gdy w ramce
    // jest Q4, to on jest źródłem orientacji. Blok pomijamy.
    p += ImuProtocol::kSizeEuler;
  }
  if (mask & ImuProtocol::kMaskQuat)
  {
    const double a = be16(p + 0) / ImuProtocol::kQuatPerUnit;
    const double b = be16(p + 2) / ImuProtocol::kQuatPerUnit;
    const double c = be16(p + 4) / ImuProtocol::kQuatPerUnit;
    const double d = be16(p + 6) / ImuProtocol::kQuatPerUnit;
    if (quat_wxyz_)
    {
      orientation_[3] = a;  // w
      orientation_[0] = b;  // x
      orientation_[1] = c;  // y
      orientation_[2] = d;  // z
    }
    else
    {
      orientation_[0] = a;
      orientation_[1] = b;
      orientation_[2] = c;
      orientation_[3] = d;
    }
    p += ImuProtocol::kSizeQuat;
  }

  calib_sys_ = (calib >> 6) & 0x03;
  calib_gyr_ = (calib >> 4) & 0x03;
  calib_acc_ = (calib >> 2) & 0x03;
  calib_mag_ = calib & 0x03;
}

size_t ImuSensor::try_parse_frame()
{
  // Nagłówek 5A 5A. Szukamy go od początku bufora; wszystko przed nim to śmieci
  // (np. ogon ramki, w którą weszliśmy w połowie po starcie).
  if (buffer_.size() < 2) { return 0; }
  if (!(buffer_[0] == ImuProtocol::kHeaderByte && buffer_[1] == ImuProtocol::kHeaderByte))
  {
    for (size_t i = 1; i + 1 < buffer_.size(); ++i)
    {
      if (buffer_[i] == ImuProtocol::kHeaderByte && buffer_[i + 1] == ImuProtocol::kHeaderByte)
      {
        return i;  // skonsumuj śmieci przed nagłówkiem
      }
    }
    return buffer_.size() - 1;  // zostaw ostatni bajt, może być połową nagłówka
  }

  if (buffer_.size() < 4) { return 0; }  // czekamy na maskę i długość
  const uint8_t mask = buffer_[2];
  const int data_len = expected_data_size(mask);
  if (data_len == 0)
  {
    ++frames_bad_;
    return 2;  // maska bez żadnej zawartości - to nie jest nasza ramka
  }

  // 5A 5A | maska | dlugosc | dane | SGAM | suma
  const size_t frame_len = 4 + static_cast<size_t>(data_len) + 2;
  if (buffer_.size() < frame_len) { return 0; }  // czekamy na resztę

  // Suma = suma wszystkich poprzedzających bajtów mod 256.
  uint8_t sum = 0;
  for (size_t i = 0; i + 1 < frame_len; ++i) { sum = static_cast<uint8_t>(sum + buffer_[i]); }

  if (sum != buffer_[frame_len - 1])
  {
    ++frames_bad_;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "IMU: błędna suma kontrolna (ramek złych: %lu, dobrych: %lu)",
      static_cast<unsigned long>(frames_bad_), static_cast<unsigned long>(frames_ok_));
    return 2;  // przeskocz nagłówek i szukaj dalej
  }

  decode(mask, buffer_.data() + 4, buffer_[frame_len - 2]);
  ++frames_ok_;
  return frame_len;
}

void ImuSensor::poll_serial()
{
  if (fd_ < 0) { return; }

  uint8_t chunk[256];
  for (;;)
  {
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n <= 0) { break; }
    buffer_.insert(buffer_.end(), chunk, chunk + n);
    if (n < static_cast<ssize_t>(sizeof(chunk))) { break; }
  }

  // Bufor nie może rosnąć w nieskończoność, gdyby na porcie siedziało coś, co
  // nigdy nie da poprawnej ramki.
  constexpr size_t kMaxBuffer = 4096;
  if (buffer_.size() > kMaxBuffer)
  {
    buffer_.erase(buffer_.begin(), buffer_.end() - kMaxBuffer / 2);
  }

  for (;;)
  {
    const size_t consumed = try_parse_frame();
    if (consumed == 0) { break; }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(consumed));
  }
}

hardware_interface::return_type ImuSensor::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  poll_serial();
  return hardware_interface::return_type::OK;
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::ImuSensor, hardware_interface::SensorInterface)
