#include "hardware_controller/laser_sensor.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

speed_t baud_to_speed(int baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B115200;
  }
}
}  // namespace

double LaserSensor::get_param(const std::string & name, double fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end())
  {
    return fallback;
  }
  return std::stod(it->second);
}

std::string LaserSensor::get_param_str(const std::string & name, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end())
  {
    return fallback;
  }
  return it->second;
}

// Format: "ADC:metry ADC:metry ...", np. "328.77:3.69 336.00:2.93 ...".
// Kolejność węzłów w zapisie jest dowolna - sortujemy po ADC.
bool LaserSensor::load_calib_table(const std::string & spec)
{
  calib_adc_.clear();
  calib_inv_l_.clear();
  calib_slope_.clear();

  std::vector<std::pair<double, double>> nodes;   // (ADC, L_cm)
  std::istringstream stream(spec);
  std::string token;
  while (stream >> token)
  {
    const auto colon = token.find(':');
    if (colon == std::string::npos)
    {
      RCLCPP_ERROR(logger_, "calib_table: węzeł '%s' bez dwukropka", token.c_str());
      return false;
    }
    try
    {
      const double adc = std::stod(token.substr(0, colon));
      const double range_m = std::stod(token.substr(colon + 1));
      if (range_m <= 0.0)
      {
        RCLCPP_ERROR(logger_, "calib_table: węzeł '%s' ma niedodatnią odległość", token.c_str());
        return false;
      }
      nodes.emplace_back(adc, range_m * 100.0);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(logger_, "calib_table: węzeł '%s' nie parsuje się: %s", token.c_str(), e.what());
      return false;
    }
  }

  if (nodes.size() < 2)
  {
    RCLCPP_ERROR(logger_, "calib_table: potrzeba co najmniej 2 węzłów, jest %zu", nodes.size());
    return false;
  }

  std::sort(nodes.begin(), nodes.end());
  for (size_t i = 1; i < nodes.size(); ++i)
  {
    if (nodes[i].first <= nodes[i - 1].first)
    {
      RCLCPP_ERROR(
        logger_, "calib_table: dwa węzły przy tym samym ADC (%.2f)", nodes[i].first);
      return false;
    }
    // Wskazanie MUSI być monotoniczne - rosnące ADC to malejąca odległość.
    // Węzły łamiące tę zasadę oznaczają pomyłkę w pomiarze albo przepisaniu,
    // a interpolator wygładziłby to w cichy błąd.
    if (nodes[i].second >= nodes[i - 1].second)
    {
      RCLCPP_ERROR(
        logger_, "calib_table: niemonotoniczne - ADC %.2f -> %.2f cm, a ADC %.2f -> %.2f cm",
        nodes[i - 1].first, nodes[i - 1].second, nodes[i].first, nodes[i].second);
      return false;
    }
  }

  calib_adc_.reserve(nodes.size());
  calib_inv_l_.reserve(nodes.size());
  for (const auto & node : nodes)
  {
    calib_adc_.push_back(node.first);
    calib_inv_l_.push_back(1.0 / node.second);   // interpolujemy 1/L, patrz nagłówek
  }
  compute_pchip_slopes();
  return true;
}

// Nachylenia metodą Fritscha-Carlsona: w węzłach wewnętrznych ważona średnia
// harmoniczna sąsiednich ilorazów różnicowych (zero przy zmianie znaku), na
// brzegach jednostronna formuła trzypunktowa z ograniczeniem monotoniczności.
// To ten sam algorytm co scipy.interpolate.PchipInterpolator - porównane
// numerycznie na naszych węzłach, różnica poniżej 0,1 mm na całym zakresie.
void LaserSensor::compute_pchip_slopes()
{
  const size_t n = calib_adc_.size();
  calib_slope_.assign(n, 0.0);

  std::vector<double> h(n - 1), delta(n - 1);
  for (size_t i = 0; i + 1 < n; ++i)
  {
    h[i] = calib_adc_[i + 1] - calib_adc_[i];
    delta[i] = (calib_inv_l_[i + 1] - calib_inv_l_[i]) / h[i];
  }

  for (size_t i = 1; i + 1 < n; ++i)
  {
    if (delta[i - 1] * delta[i] <= 0.0)
    {
      calib_slope_[i] = 0.0;   // ekstremum lokalne - płasko, żeby nie przestrzelić
      continue;
    }
    const double w1 = 2.0 * h[i] + h[i - 1];
    const double w2 = h[i] + 2.0 * h[i - 1];
    calib_slope_[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
  }

  const auto edge = [](double h0, double h1, double d0, double d1) {
    double slope = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
    if (slope * d0 <= 0.0)
    {
      return 0.0;
    }
    if (d0 * d1 <= 0.0 && std::abs(slope) > std::abs(3.0 * d0))
    {
      return 3.0 * d0;
    }
    return slope;
  };

  if (n == 2)
  {
    calib_slope_[0] = delta[0];
    calib_slope_[1] = delta[0];
    return;
  }
  calib_slope_[0] = edge(h[0], h[1], delta[0], delta[1]);
  calib_slope_[n - 1] = edge(h[n - 2], h[n - 3], delta[n - 2], delta[n - 3]);
}

// Wielomian Hermite'a na przedziale zawierającym adc. Poza tabelą NaN -
// PCHIP przedłużyłby brzegowy wielomian sześcienny i potrafi uciec o metry.
double LaserSensor::interpolate_inv_l(double adc) const
{
  if (calib_adc_.size() < 2 || adc < calib_adc_.front() || adc > calib_adc_.back())
  {
    return kNaN;
  }

  const auto upper = std::upper_bound(calib_adc_.begin(), calib_adc_.end(), adc);
  size_t i = static_cast<size_t>(std::distance(calib_adc_.begin(), upper));
  i = (i == 0) ? 0 : i - 1;
  if (i + 1 >= calib_adc_.size())
  {
    i = calib_adc_.size() - 2;
  }

  const double h = calib_adc_[i + 1] - calib_adc_[i];
  const double t = (adc - calib_adc_[i]) / h;
  const double t2 = t * t;
  const double t3 = t2 * t;

  return calib_inv_l_[i] * (2.0 * t3 - 3.0 * t2 + 1.0) +
         h * calib_slope_[i] * (t3 - 2.0 * t2 + t) +
         calib_inv_l_[i + 1] * (-2.0 * t3 + 3.0 * t2) +
         h * calib_slope_[i + 1] * (t3 - t2);
}

hardware_interface::CallbackReturn LaserSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters.at("device_port");
  baud_rate_ = static_cast<int>(get_param("baud_rate", 115200));
  sensor_name_ = info_.sensors.at(0).name;

  const std::string table_spec = get_param_str("calib_table", "");
  if (!table_spec.empty() && !load_calib_table(table_spec))
  {
    // Zła tabela to błąd konfiguracji, nie powód do cichego zjechania na
    // wartości katalogowe - te mylą się o kilkanaście cm i nikt by nie zauważył.
    RCLCPP_ERROR(logger_, "calib_table podane, ale niepoprawne - przerywam.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  calib_a_ = get_param("calib_a", calib_a_);
  calib_b_ = get_param("calib_b", calib_b_);
  vcc_ = get_param("vcc", vcc_);
  adc_max_ = get_param("adc_max", adc_max_);
  min_range_ = get_param("min_range", min_range_);
  max_range_ = get_param("max_range", max_range_);
  settle_ms_ = static_cast<int>(get_param("settle_ms", settle_ms_));

  state_range_ = kNaN;
  state_adc_ = kNaN;
  state_adc_spread_ = kNaN;
  state_sample_time_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LaserSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(sensor_name_, "range", &state_range_);
  interfaces.emplace_back(sensor_name_, "adc", &state_adc_);
  interfaces.emplace_back(sensor_name_, "adc_spread", &state_adc_spread_);
  interfaces.emplace_back(sensor_name_, "sample_time", &state_sample_time_);
  return interfaces;
}

hardware_interface::CallbackReturn LaserSensor::on_configure(const rclcpp_lifecycle::State &)
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

  cfmakeraw(&tty);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  // HUPCL opuszcza DTR przy zamknięciu portu, czyli resetuje Nano także wtedy,
  // gdy tylko odłączamy się od czujnika. Nic nam po tym resecie.
  tty.c_cflag &= ~HUPCL;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcsetattr nieudane na %s", device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Nano wstaje po resecie z DTR - przeczekaj bootloader i wyrzuć wszystko, co
  // zdążyło wpaść (banner "#LSR ready" i ewentualny ogryzek pierwszej ramki).
  if (settle_ms_ > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms_));
  }
  tcflush(fd_, TCIFLUSH);
  line_buffer_.clear();

  if (calib_adc_.empty())
  {
    RCLCPP_WARN(
      logger_,
      "Dalmierz na %s @%d: BRAK calib_table, jadę na wartościach katalogowych "
      "V = %.4f/L_cm + %.4f. Mylą się o kilkanaście cm - do rekonstrukcji podaj tabelę.",
      device_port_.c_str(), baud_rate_, calib_a_, calib_b_);
  }
  else
  {
    RCLCPP_INFO(
      logger_, "Dalmierz na %s @%d, tabela PCHIP: %zu węzłów, ADC %.1f-%.1f = %.2f-%.2f m",
      device_port_.c_str(), baud_rate_, calib_adc_.size(), calib_adc_.front(),
      calib_adc_.back(), 1.0 / calib_inv_l_.back() / 100.0,
      1.0 / calib_inv_l_.front() / 100.0);
  }
  RCLCPP_INFO(
    logger_, "Zakres raportowany: %.2f-%.2f m, poza nim NaN", min_range_, max_range_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LaserSensor::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  poll_serial(time);
  return hardware_interface::return_type::OK;
}

void LaserSensor::poll_serial(const rclcpp::Time & time)
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
    // Znacznik czasu to początek bieżącego cyklu ros2_control. Ramka mogła
    // wpaść do bufora jądra wcześniej, ale błąd jest ograniczony jednym okresem
    // pętli (20 ms przy 50 Hz) - mniej niż okno mediany w samym firmware.
    process_line(line, time);
  }
}

void LaserSensor::process_line(const std::string & line, const rclcpp::Time & time)
{
  // Linie diagnostyczne firmware'u zaczynają się od '#'.
  if (line.empty() || line[0] == '#')
  {
    return;
  }

  std::vector<std::string> fields;
  size_t start = 0;
  while (true)
  {
    const size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    if (comma == std::string::npos)
    {
      break;
    }
    start = comma + 1;
  }

  // LSR,<mediana>,<min>,<max>,<n>
  if (fields.size() != 5 || fields[0] != "LSR")
  {
    return;
  }

  try
  {
    const double median = std::stod(fields[1]);
    const double lo = std::stod(fields[2]);
    const double hi = std::stod(fields[3]);

    state_adc_ = median;
    state_adc_spread_ = hi - lo;
    state_range_ = adc_to_range(median);
    state_sample_time_ = time.seconds();
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger_, "Nie udało się sparsować ramki '%s': %s", line.c_str(), e.what());
  }
}

double LaserSensor::adc_to_range(double adc) const
{
  double range_m = kNaN;

  if (!calib_adc_.empty())
  {
    const double inv_l = interpolate_inv_l(adc);
    if (!std::isfinite(inv_l) || inv_l <= 0.0)
    {
      // Poza tabelą kalibracyjną - nie zgadujemy, patrz komentarz w nagłówku.
      return kNaN;
    }
    range_m = 1.0 / inv_l / 100.0;
  }
  else
  {
    const double volts = adc * vcc_ / adc_max_;
    const double denom = volts - calib_b_;
    if (denom <= 0.0)
    {
      // Napięcie poniżej asymptoty modelu: brak echa albo cel dalej niż zasięg.
      return kNaN;
    }
    range_m = calib_a_ / denom / 100.0;
  }

  if (range_m < min_range_ || range_m > max_range_)
  {
    // Poza wiarygodnym zakresem. Poniżej min_range_ leży strefa zawinięcia
    // krzywej, gdzie ta sama wartość odpowiada dwóm odległościom; powyżej
    // max_range_ czujnikowi kończy się rozdzielczość. Patrz nagłówek.
    return kNaN;
  }
  return range_m;
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::LaserSensor, hardware_interface::SensorInterface)
