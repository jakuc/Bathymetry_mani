#include "hardware_controller/m703a_laser_sensor.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr uint8_t kEsc = 0x1B;
constexpr double kWrapUs = 4294967296.0;      // 2^32 mikrosekund

// Tabela baudów mostka - MUSI się zgadzać z BAUDS[] w firmware/jrt_bridge.
// Mostek dostaje INDEKS, nie prędkość, więc rozjazd tych list ustawia zupełnie
// inny baud bez żadnego komunikatu.
const std::vector<int> kBridgeBauds = {19200, 9600, 38400, 57600, 4800, 2400, 14400, 115200};

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

double M703aLaserSensor::get_param(const std::string & name, double fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  return (it == info_.hardware_parameters.end()) ? fallback : std::stod(it->second);
}

std::string M703aLaserSensor::get_param_str(const std::string & name, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  return (it == info_.hardware_parameters.end()) ? fallback : it->second;
}

hardware_interface::CallbackReturn M703aLaserSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters.at("device_port");
  sensor_name_ = info_.sensors.at(0).name;

  usb_baud_ = static_cast<int>(get_param("usb_baud", usb_baud_));
  module_baud_ = static_cast<int>(get_param("module_baud", module_baud_));
  settle_ms_ = static_cast<int>(get_param("settle_ms", settle_ms_));
  min_range_ = get_param("min_range", min_range_);
  max_range_ = get_param("max_range", max_range_);
  anchor_offset_s_ = get_param("anchor_offset_ms", 0.0) / 1000.0;
  restart_after_ms_ = static_cast<int>(get_param("restart_after_ms", restart_after_ms_));
  clock_window_s_ = get_param("clock_window_s", clock_window_s_);
  clock_buckets_ = static_cast<int>(get_param("clock_buckets", clock_buckets_));
  measure_mode_ = get_param_str("measure_mode", measure_mode_);

  if (measure_mode_ != "F" && measure_mode_ != "D" && measure_mode_ != "M")
  {
    RCLCPP_ERROR(logger_, "measure_mode='%s' - dozwolone: F (fast), D (auto), M (slow)",
                 measure_mode_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (clock_buckets_ < 3)
  {
    RCLCPP_ERROR(logger_, "clock_buckets=%d - obwiednia potrzebuje co najmniej 3", clock_buckets_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  state_range_ = kNaN;
  state_signal_quality_ = kNaN;
  state_status_code_ = kNaN;
  state_sample_time_ = 0.0;
  state_shot_start_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> M703aLaserSensor::export_state_interfaces()
{
  // Te same nazwy co w JrtLaserSensor - LaserBroadcaster i kolektor nie muszą
  // wiedzieć, który dalmierz siedzi na głowicy.
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(sensor_name_, "range", &state_range_);
  interfaces.emplace_back(sensor_name_, "signal_quality", &state_signal_quality_);
  interfaces.emplace_back(sensor_name_, "status_code", &state_status_code_);
  interfaces.emplace_back(sensor_name_, "sample_time", &state_sample_time_);
  interfaces.emplace_back(sensor_name_, "shot_start", &state_shot_start_);
  return interfaces;
}

bool M703aLaserSensor::open_port()
{
  fd_ = ::open(device_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Nie można otworzyć portu %s", device_port_.c_str());
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcgetattr nieudane na %s", device_port_.c_str());
    return false;
  }

  const speed_t speed = baud_to_speed(usb_baud_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CREAD | CLOCAL;
  // HUPCL opuszczałby DTR przy zamknięciu, czyli resetował mostek także wtedy,
  // gdy tylko się od niego odłączamy. Nic nam po tym resecie.
  tty.c_cflag &= ~HUPCL;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcsetattr nieudane na %s", device_port_.c_str());
    return false;
  }
  return true;
}

void M703aLaserSensor::write_raw(const std::vector<uint8_t> & bytes)
{
  if (fd_ < 0 || bytes.empty())
  {
    return;
  }
  if (::write(fd_, bytes.data(), bytes.size()) < 0)
  {
    RCLCPP_WARN_THROTTLE(logger_, clock_throttle_, 5000, "zapis na %s nieudany", device_port_.c_str());
  }
}

void M703aLaserSensor::bridge_cmd(uint8_t cmd) { write_raw({kEsc, kEsc, cmd}); }
void M703aLaserSensor::bridge_cmd(uint8_t cmd, uint8_t arg) { write_raw({kEsc, kEsc, cmd, arg}); }

size_t M703aLaserSensor::drain(std::string & sink, int wait_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
  size_t total = 0;
  char buf[256];
  while (std::chrono::steady_clock::now() < deadline)
  {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0)
    {
      sink.append(buf, static_cast<size_t>(n));
      total += static_cast<size_t>(n);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return total;
}

void M703aLaserSensor::pump_lines(std::vector<std::string> & out)
{
  char buf[256];
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0)
  {
    rx_.append(buf, static_cast<size_t>(n));
  }

  size_t pos;
  while ((pos = rx_.find('\n')) != std::string::npos)
  {
    std::string line = rx_.substr(0, pos);
    rx_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    if (!line.empty())
    {
      out.push_back(line);
    }
  }
  // Linia bez końca nie może rosnąć w nieskończoność przy zakłóconej transmisji.
  if (rx_.size() > 512)
  {
    rx_.clear();
    ++bad_lines_;
  }
}

bool M703aLaserSensor::parse_stamped(const std::string & line, uint32_t & t_first, uint32_t & t_last,
                                     std::string & out_body)
{
  if (line.empty() || line[0] != '$')
  {
    return false;
  }
  const size_t c1 = line.find(',', 1);
  if (c1 == std::string::npos) { return false; }
  const size_t c2 = line.find(',', c1 + 1);
  if (c2 == std::string::npos) { return false; }

  char * end = nullptr;
  const unsigned long long a = std::strtoull(line.c_str() + 1, &end, 10);
  if (end != line.c_str() + c1) { return false; }
  const unsigned long long b = std::strtoull(line.c_str() + c1 + 1, &end, 10);
  if (end != line.c_str() + c2) { return false; }

  t_first = static_cast<uint32_t>(a);
  t_last = static_cast<uint32_t>(b);
  out_body = line.substr(c2 + 1);
  return true;
}

bool M703aLaserSensor::parse_measurement(const std::string & body, double & out_m, int & out_sq)
{
  // Format: "[litera]: <dystans>m,<SQ>"; dystans poniżej 10 m ma spację zamiast
  // cyfry dziesiątek (moduł trzyma stałą długość linii).
  const size_t m_pos = body.find('m');
  if (m_pos == std::string::npos) { return false; }
  const size_t colon = body.find(':');
  const size_t start = (colon == std::string::npos) ? 0 : colon + 1;
  if (m_pos <= start) { return false; }

  const std::string num = body.substr(start, m_pos - start);
  char * end = nullptr;
  const double meters = std::strtod(num.c_str(), &end);
  if (end == num.c_str()) { return false; }

  const size_t comma = body.find(',', m_pos);
  if (comma == std::string::npos) { return false; }
  const long sq = std::strtol(body.c_str() + comma + 1, &end, 10);
  if (end == body.c_str() + comma + 1) { return false; }

  out_m = meters;
  out_sq = static_cast<int>(sq);
  return true;
}

double M703aLaserSensor::unwrap_nano(uint32_t raw_us)
{
  if (!have_epoch_)
  {
    have_epoch_ = true;
    last_raw_us_ = raw_us;
    nano_epoch_ = 0.0;
    nano_first_ = static_cast<double>(raw_us) / 1e6;
    return 0.0;
  }
  if (raw_us < last_raw_us_ && (last_raw_us_ - raw_us) > 0x80000000u)
  {
    nano_epoch_ += kWrapUs / 1e6;      // przewinięcie licznika micros()
  }
  last_raw_us_ = raw_us;
  return static_cast<double>(raw_us) / 1e6 + nano_epoch_ - nano_first_;
}

// Relacja zegarów z DOLNEJ OBWIEDNI - uzasadnienie w nagłówku klasy.
void M703aLaserSensor::clock_add(double nano_s, double host_s)
{
  clock_.emplace_back(nano_s, host_s - nano_s);
  while (!clock_.empty() && nano_s - clock_.front().first > clock_window_s_)
  {
    clock_.pop_front();
  }
  if (clock_.size() < 8)
  {
    // Za mało na obwiednię: bierzemy samo minimum jako stałe przesunięcie.
    // Dryf zostaje przy zerze, więc pierwsze sekundy po starcie są obarczone
    // błędem rosnącym 3 ms/s - i tak kilkanaście razy mniej niż jitter USB.
    double best = clock_.front().second;
    for (const auto & s : clock_) { best = std::min(best, s.second); }
    clock_offset_ = best;
    clock_drift_ = 0.0;
    clock_valid_ = true;
    return;
  }

  const double x0 = clock_.front().first;
  const double span = clock_.back().first - x0;
  if (span <= 1e-6) { return; }

  // Minimum w każdym koszyku czasu, potem prosta przez te minima.
  std::vector<double> bx(clock_buckets_, 0.0), by(clock_buckets_, 0.0);
  std::vector<bool> have(clock_buckets_, false);
  for (const auto & s : clock_)
  {
    int k = static_cast<int>((s.first - x0) / span * clock_buckets_);
    k = std::min(std::max(k, 0), clock_buckets_ - 1);
    if (!have[k] || s.second < by[k])
    {
      have[k] = true;
      bx[k] = s.first;
      by[k] = s.second;
    }
  }

  double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
  int n = 0;
  for (int k = 0; k < clock_buckets_; ++k)
  {
    if (!have[k]) { continue; }
    sx += bx[k]; sy += by[k]; sxx += bx[k] * bx[k]; sxy += bx[k] * by[k];
    ++n;
  }
  if (n < 3) { return; }

  const double denom = n * sxx - sx * sx;
  if (std::fabs(denom) < 1e-12) { return; }
  clock_drift_ = (n * sxy - sx * sy) / denom;
  clock_offset_ = (sy - clock_drift_ * sx) / n;
  clock_valid_ = true;
}

bool M703aLaserSensor::clock_ready() const { return clock_valid_; }

double M703aLaserSensor::clock_to_host(double nano_s) const
{
  return nano_s + clock_drift_ * nano_s + clock_offset_;
}

void M703aLaserSensor::start_stream()
{
  bridge_cmd(0x54, 1);                                   // stemple czasu na Nano
  bridge_cmd(0x4E, 0);                                   // nCTRL nisko = tryb ciągły
  write_raw({static_cast<uint8_t>(measure_mode_[0])});   // 'F' / 'D' / 'M'
  skip_first_ = true;
}

hardware_interface::CallbackReturn M703aLaserSensor::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_port())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mostek wstaje po resecie z DTR - przeczekaj bootloader i wyrzuć baner.
  if (settle_ms_ > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms_));
  }
  tcflush(fd_, TCIFLUSH);
  rx_.clear();

  if (std::find(kBridgeBauds.begin(), kBridgeBauds.end(), module_baud_) == kBridgeBauds.end())
  {
    RCLCPP_ERROR(logger_, "module_baud=%d nie występuje w tabeli mostka", module_baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  const uint8_t baud_idx = static_cast<uint8_t>(
    std::distance(kBridgeBauds.begin(), std::find(kBridgeBauds.begin(), kBridgeBauds.end(), module_baud_)));

  std::string junk;
  bridge_cmd(0x4B, baud_idx);            // baud modułu
  drain(junk, 200);
  bridge_cmd(0x50, 1);                   // PWR_ON modułu
  drain(junk, 500);
  bridge_cmd(0x4E, 1);                   // nCTRL wysoko - gdyby został ciągły z poprzedniego razu
  drain(junk, 100);
  bridge_cmd(0x54, 1);                   // stemple czasu
  junk.clear();
  drain(junk, 200);

  // DOWÓD ŻYCIA. Komenda 'S' zwraca temperaturę i napięcie modułu i NIE odpala
  // lasera, więc to tani test. Bez niego stack wstawałby cicho i sypał NaN-ami,
  // a przyczyny (brak PWR_ON, zły baud, martwy mostek) byłyby nie do odróżnienia.
  tcflush(fd_, TCIFLUSH);
  rx_.clear();
  junk.clear();
  write_raw({'S'});
  drain(junk, 600);
  if (junk.find("`C") == std::string::npos && junk.find('V') == std::string::npos)
  {
    RCLCPP_ERROR(
      logger_,
      "Dalmierz M703A na %s nie odpowiada na 'S'. Sprawdź kolejno: PWR_ON (D4 mostka), "
      "baud modułu (%d), zasilanie 3,0 V modułu, TXD modułu -> D2 i RXD modułu <- D3. "
      "Odebrano: '%s'",
      device_port_.c_str(), module_baud_, junk.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    std::string status = junk;
    const auto cut = status.find_last_not_of("\r\n");
    if (cut != std::string::npos) { status.resize(cut + 1); }
    RCLCPP_INFO(logger_, "Dalmierz M703A na %s @%d, tryb ciągły %s, moduł melduje: %s",
                device_port_.c_str(), module_baud_, measure_mode_.c_str(), status.c_str());
  }
  RCLCPP_INFO(logger_, "Kotwica czasu: stempel Nano %+.1f ms; zakres %.2f-%.2f m, poza nim NaN",
              anchor_offset_s_ * 1000.0, min_range_, max_range_);

  clock_.clear();
  clock_valid_ = false;
  clock_drift_ = 0.0;
  clock_offset_ = 0.0;
  have_epoch_ = false;
  have_line_ = false;
  lines_ = bad_lines_ = restarts_ = 0;
  tcflush(fd_, TCIFLUSH);
  rx_.clear();
  start_stream();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn M703aLaserSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    bridge_cmd(0x4E, 1);                 // nCTRL wysoko = stop trybu ciągłego
    std::string junk;
    drain(junk, 150);
    write_raw({'C'});                    // laser off
    drain(junk, 150);
    bridge_cmd(0x50, 0);                 // PWR_ON w dół - moduł śpi
    drain(junk, 100);
    ::close(fd_);
    fd_ = -1;
  }
  RCLCPP_INFO(
    logger_, "Dalmierz M703A: %lu pomiarów, %lu linii odrzuconych, %lu wznowień strumienia, "
    "dryf zegara Nano %+.0f ppm",
    static_cast<unsigned long>(lines_), static_cast<unsigned long>(bad_lines_),
    static_cast<unsigned long>(restarts_), clock_drift_ * 1e6);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type M703aLaserSensor::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<std::string> lines;
  pump_lines(lines);

  for (const std::string & line : lines)
  {
    // Baner = Nano się zresetowało. Po resecie stemple są WYŁĄCZONE, a nCTRL
    // wraca do stanu spoczynkowego, więc sam strumień by nie wrócił.
    if (line.rfind("#JRTBRIDGE", 0) == 0)
    {
      ++restarts_;
      RCLCPP_WARN(logger_, "Mostek się zresetował (%s) - wznawiam tryb ciągły", line.c_str());
      have_epoch_ = false;
      clock_.clear();
      clock_valid_ = false;
      start_stream();
      continue;
    }
    if (!line.empty() && line[0] == '#')
    {
      continue;                          // potwierdzenia komend mostka
    }

    uint32_t t_first = 0, t_last = 0;
    std::string body;
    if (!parse_stamped(line, t_first, t_last, body))
    {
      ++bad_lines_;
      RCLCPP_WARN_THROTTLE(logger_, clock_throttle_, 5000,
                           "Linia bez stempla: '%s' (%lu takich)", line.c_str(),
                           static_cast<unsigned long>(bad_lines_));
      continue;
    }

    const double nano_s = unwrap_nano(t_first);
    // Czas ostatniego bajtu tej samej linii. Różnica uint32 liczy się poprawnie
    // także przez przewinięcie licznika (arytmetyka modulo 2^32).
    const double nano_last_s = nano_s + static_cast<double>(static_cast<uint32_t>(t_last - t_first)) / 1e6;
    have_line_ = true;
    last_line_stamp_ = time;

    if (skip_first_)
    {
      // Patrz komentarz przy skip_first_: pierwsza linia po komendzie trybu
      // niesie chwilę przyjęcia komendy, nie pomiaru.
      skip_first_ = false;
      continue;
    }

    // Relację zegarów liczymy od OSTATNIEGO bajtu, bo dopiero po nim mostek
    // wysyła linię do hosta - czas przyjścia na hoście jest więc zawsze >= t_last,
    // a nie >= t_first. Liczone od t_first obwiednia zawierałaby czas nadawania
    // linii (~7,3 ms dla pomiaru), a ten ZALEŻY OD DŁUGOŚCI: linie "Er08" są
    // o ~7 bajtów krótsze, więc gdy trafiały do okna, obwiednia przeskakiwała
    // na nie i przesuwała cały czas o ~3,6 ms (0,1 st przy 24 st/s).
    clock_add(nano_last_s, time.seconds());
    if (!clock_ready())
    {
      continue;
    }

    const double t_measure = clock_to_host(nano_s);

    double meters = 0.0;
    int sq = 0;
    if (parse_measurement(body, meters, sq))
    {
      state_signal_quality_ = static_cast<double>(sq);
      state_status_code_ = 0.0;
      // Poza wiarygodnym zakresem raportujemy NaN, a nie liczbę: cichy błąd przy
      // rekonstrukcji chmury wygląda jak poprawny punkt.
      state_range_ = (meters < min_range_ || meters > max_range_) ? kNaN : meters;
      ++lines_;
    }
    else if (body.find("Er") != std::string::npos)
    {
      // "Er08!" - moduł zmierzył i wie, że się nie udało (za słaby cel).
      // To PEŁNOPRAWNY wynik: stemplujemy go, żeby broadcaster opublikował NaN
      // zamiast milczeć, bo inaczej luka w chmurze nie ma wyjaśnienia.
      state_range_ = kNaN;
      state_signal_quality_ = kNaN;
      const size_t p = body.find("Er");
      state_status_code_ = std::strtod(body.c_str() + p + 2, nullptr);
      ++lines_;
    }
    else
    {
      ++bad_lines_;
      continue;                          // odpowiedzi na S/V - nie pomiar
    }

    state_shot_start_ = t_measure;
    state_sample_time_ = t_measure + anchor_offset_s_;
  }

  // Cisza dłuższa niż restart_after_ms_: moduł wypadł z trybu ciągłego albo
  // ktoś podniósł nCTRL. Wznowienie jest tanie (trzy bajty) i nie blokuje pętli.
  if (have_line_ && (time - last_line_stamp_).seconds() * 1000.0 > restart_after_ms_)
  {
    ++restarts_;
    RCLCPP_WARN(logger_, "Brak pomiarów przez %d ms - wznawiam tryb ciągły (%lu raz)",
                restart_after_ms_, static_cast<unsigned long>(restarts_));
    last_line_stamp_ = time;
    start_stream();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::M703aLaserSensor, hardware_interface::SensorInterface)
