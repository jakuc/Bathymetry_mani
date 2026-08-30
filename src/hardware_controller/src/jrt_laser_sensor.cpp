#include "hardware_controller/jrt_laser_sensor.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdio>
#include <numeric>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

constexpr uint8_t kHeadOk = 0xAA;
constexpr uint8_t kHeadErr = 0xEE;
constexpr uint8_t kEsc = 0x1B;
constexpr uint8_t kWake = 0x55;      // auto-baud; moduł odpowiada swoim adresem
constexpr uint8_t kStop = 0x58;      // wyjście z trybu ciągłego

constexpr uint16_t kRegStatus = 0x0000;
constexpr uint16_t kRegSerial = 0x000E;
constexpr uint16_t kRegResult = 0x0022;

// Tabela baudów mostka - MUSI się zgadzać z BAUDS[] w firmware/jrt_bridge.
// Mostek dostaje INDEKS, nie prędkość, więc rozjazd tych dwóch list daje
// ustawienie zupełnie innego baudu bez żadnego komunikatu.
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

double JrtLaserSensor::get_param(const std::string & name, double fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  return (it == info_.hardware_parameters.end()) ? fallback : std::stod(it->second);
}

std::string JrtLaserSensor::get_param_str(const std::string & name, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  return (it == info_.hardware_parameters.end()) ? fallback : it->second;
}

std::vector<uint8_t> JrtLaserSensor::make_frame(const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 2);
  frame.push_back(kHeadOk);
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint32_t sum = std::accumulate(payload.begin(), payload.end(), 0u);
  frame.push_back(static_cast<uint8_t>(sum & 0xFF));
  return frame;
}

hardware_interface::CallbackReturn JrtLaserSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters.at("device_port");
  sensor_name_ = info_.sensors.at(0).name;

  usb_baud_ = static_cast<int>(get_param("usb_baud", usb_baud_));
  module_baud_ = static_cast<int>(get_param("module_baud", module_baud_));
  period_ms_ = static_cast<int>(get_param("period_ms", period_ms_));
  settle_ms_ = static_cast<int>(get_param("settle_ms", settle_ms_));
  min_range_ = get_param("min_range", min_range_);
  max_range_ = get_param("max_range", max_range_);
  measure_mode_ = get_param_str("measure_mode", measure_mode_);
  recover_after_misses_ = static_cast<int>(get_param("recover_after_misses", recover_after_misses_));

  // Payload komendy pomiaru różni się JEDNYM bajtem trybu.
  uint8_t mode_byte;
  if (measure_mode_ == "fast")       { mode_byte = 0x02; }
  else if (measure_mode_ == "slow")  { mode_byte = 0x01; }
  else if (measure_mode_ == "auto")  { mode_byte = 0x00; }
  else
  {
    RCLCPP_ERROR(logger_, "measure_mode='%s' - dozwolone: fast, slow, auto", measure_mode_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  measure_frame_ = make_frame({0x00, 0x00, 0x20, 0x00, 0x01, 0x00, mode_byte});

  // Domyślny timeout zależy od trybu: slow potrafi mielić 3,3 s, więc wspólna
  // stała musiałaby być albo za długa dla fast, albo za krótka dla slow.
  const int default_timeout = (measure_mode_ == "slow") ? 5000 : 1500;
  timeout_ms_ = static_cast<int>(get_param("timeout_ms", default_timeout));

  state_range_ = kNaN;
  state_signal_quality_ = kNaN;
  state_status_code_ = kNaN;
  state_sample_time_ = 0.0;
  state_shot_start_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JrtLaserSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(sensor_name_, "range", &state_range_);
  interfaces.emplace_back(sensor_name_, "signal_quality", &state_signal_quality_);
  interfaces.emplace_back(sensor_name_, "status_code", &state_status_code_);
  interfaces.emplace_back(sensor_name_, "sample_time", &state_sample_time_);
  interfaces.emplace_back(sensor_name_, "shot_start", &state_shot_start_);
  return interfaces;
}

bool JrtLaserSensor::open_port()
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
  // HUPCL opuszcza DTR przy zamknięciu, czyli resetuje mostek także wtedy, gdy
  // tylko odłączamy się od czujnika. Nic nam po tym resecie.
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

void JrtLaserSensor::write_raw(const std::vector<uint8_t> & bytes)
{
  if (fd_ < 0 || bytes.empty())
  {
    return;
  }
  const ssize_t written = ::write(fd_, bytes.data(), bytes.size());
  if (written < 0)
  {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 5000, "zapis na %s nieudany", device_port_.c_str());
  }
}

void JrtLaserSensor::bridge_cmd(uint8_t cmd)
{
  write_raw({kEsc, kEsc, cmd});
}

void JrtLaserSensor::bridge_cmd(uint8_t cmd, uint8_t arg)
{
  write_raw({kEsc, kEsc, cmd, arg});
}

// Ramka trafia do mostka jako 1B 1B 'W' <len> <dane>. Mostek zbiera ją do RAM
// przy działających przerwaniach i dopiero potem nadaje bez przerw między
// bajtami. Bez tego 9-bajtowe ramki zapisu rozjeżdżały się, a moduł odpowiadał
// kodem 0x0081 ("nieprawidłowa ramka") - objawowo nie do odróżnienia od złego
// baudu (rozpoznane 2026-08-29).
void JrtLaserSensor::write_frame(const std::vector<uint8_t> & frame)
{
  std::vector<uint8_t> out{kEsc, kEsc, 0x57, static_cast<uint8_t>(frame.size())};
  out.insert(out.end(), frame.begin(), frame.end());
  write_raw(out);
}

size_t JrtLaserSensor::drain(std::vector<uint8_t> & sink, int wait_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
  size_t total = 0;
  uint8_t buf[256];
  while (std::chrono::steady_clock::now() < deadline)
  {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0)
    {
      sink.insert(sink.end(), buf, buf + n);
      total += static_cast<size_t>(n);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return total;
}

// Skanuje bufor w poszukiwaniu kompletnej ramki o poprawnej sumie. Bajty przed
// nagłówkiem odrzuca - mostek wypisuje też własne linie diagnostyczne (#BAUD,
// #PWREN), a moduł odpowiada jednym bajtem adresu na wybudzenie 0x55.
bool JrtLaserSensor::extract_frame(uint16_t & out_reg, std::vector<uint8_t> & out_payload,
                                   bool & out_is_error)
{
  size_t i = 0;
  while (i < rx_.size())
  {
    if (rx_[i] != kHeadOk && rx_[i] != kHeadErr)
    {
      ++i;
      continue;
    }
    // Nagłówek + adres + rejestr(2) + licznik słów(2) = 6 bajtów minimum,
    // zanim w ogóle wiadomo, jak długa jest ramka.
    if (rx_.size() - i < 6)
    {
      break;                      // dosypie się w kolejnym cyklu
    }
    const uint16_t words = static_cast<uint16_t>((rx_[i + 4] << 8) | rx_[i + 5]);
    const size_t data_len = 2u * words;
    const size_t total = 6 + data_len + 1;         // + suma kontrolna
    if (data_len > 32)
    {
      ++i;                        // bzdurny licznik - to nie był nagłówek
      continue;
    }
    if (rx_.size() - i < total)
    {
      break;
    }

    uint32_t sum = 0;
    for (size_t k = i + 1; k < i + total - 1; ++k)
    {
      sum += rx_[k];
    }
    if (static_cast<uint8_t>(sum & 0xFF) != rx_[i + total - 1])
    {
      ++i;                        // suma się nie zgadza - szukaj dalej
      continue;
    }

    out_is_error = (rx_[i] == kHeadErr);
    out_reg = static_cast<uint16_t>((rx_[i + 2] << 8) | rx_[i + 3]);
    out_payload.assign(rx_.begin() + i + 6, rx_.begin() + i + 6 + data_len);
    rx_.erase(rx_.begin(), rx_.begin() + i + total);
    return true;
  }

  // Nic sensownego - wyrzuć śmieci sprzed ostatniego kandydata na nagłówek,
  // żeby bufor nie puchł w nieskończoność przy zakłóconej transmisji.
  if (i > 0)
  {
    rx_.erase(rx_.begin(), rx_.begin() + std::min(i, rx_.size()));
  }
  if (rx_.size() > 512)
  {
    rx_.clear();
  }
  return false;
}

bool JrtLaserSensor::query_blocking(const std::vector<uint8_t> & frame, uint16_t expect_reg,
                                    std::vector<uint8_t> & payload, int timeout_ms)
{
  rx_.clear();
  tcflush(fd_, TCIFLUSH);
  write_frame(frame);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    drain(rx_, 20);
    uint16_t reg = 0;
    bool is_error = false;
    std::vector<uint8_t> got;
    while (extract_frame(reg, got, is_error))
    {
      if (!is_error && reg == expect_reg)
      {
        payload = got;
        return true;
      }
    }
  }
  return false;
}

uint8_t JrtLaserSensor::bridge_baud_index() const
{
  const auto it = std::find(kBridgeBauds.begin(), kBridgeBauds.end(), module_baud_);
  return static_cast<uint8_t>(std::distance(kBridgeBauds.begin(), it));
}

// Sekwencja rozruchowa rozłożona na cykle read(). Każdy etap tylko WYSYŁA i
// ustawia termin, po którym wolno przejść dalej - nigdzie nie ma sleepa, bo ta
// funkcja biegnie w pętli 50 Hz razem ze sterowaniem serwami.
bool JrtLaserSensor::step_recovery(const rclcpp::Time & time)
{
  if (time < recover_deadline_)
  {
    return false;
  }

  switch (recover_stage_)
  {
    case 0:
      // Status mostka. Odpowiedź "#JRTBRIDGE baud=..." rozstrzyga, czy Nano
      // się zresetowało (wstaje z 19200) - patrz komentarz w nagłówku.
      rx_.clear();
      bridge_cmd(0x3F);
      recover_deadline_ = time + rclcpp::Duration::from_seconds(0.3);
      break;

    case 1:
    {
      std::string banner(rx_.begin(), rx_.end());
      const auto cut = banner.find_last_not_of("\r\n");
      if (cut != std::string::npos) { banner.resize(cut + 1); }
      if (!banner.empty())
      {
        RCLCPP_WARN(logger_, "Odtwarzanie łącza, mostek melduje: %s", banner.c_str());
      }
      else
      {
        RCLCPP_WARN(logger_, "Odtwarzanie łącza: mostek NIE ODPOWIADA na zapytanie o status");
      }
      rx_.clear();
      bridge_cmd(0x4B, bridge_baud_index());     // baud modułu od nowa
      recover_deadline_ = time + rclcpp::Duration::from_seconds(0.3);
      break;
    }

    case 2:
      bridge_cmd(0x50, 1);                       // pełny cykl PWREN + nRST
      recover_deadline_ = time + rclcpp::Duration::from_seconds(0.8);
      break;

    case 3:
      write_raw({kWake});                        // auto-baud
      recover_deadline_ = time + rclcpp::Duration::from_seconds(0.3);
      break;

    default:
      tcflush(fd_, TCIFLUSH);
      rx_.clear();
      recover_stage_ = 0;
      consecutive_misses_ = 0;
      return true;
  }

  ++recover_stage_;
  return false;
}

hardware_interface::CallbackReturn JrtLaserSensor::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_port())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mostek wstaje po resecie z DTR - przeczekaj bootloader i wyrzuć banner.
  if (settle_ms_ > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms_));
  }
  tcflush(fd_, TCIFLUSH);
  rx_.clear();

  // 1. Baud modułu. Mostek przyjmuje INDEKS z własnej tabeli.
  if (std::find(kBridgeBauds.begin(), kBridgeBauds.end(), module_baud_) == kBridgeBauds.end())
  {
    RCLCPP_ERROR(logger_, "module_baud=%d nie występuje w tabeli mostka", module_baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  bridge_cmd(0x4B, bridge_baud_index());
  std::vector<uint8_t> junk;
  drain(junk, 300);

  // 2. Zasilanie modułu. PWREN domyślnie NISKI = moduł wyłączony; mostek robi
  //    pełny cykl power-down -> power-up -> zwolnienie nRST -> ~300 ms bootu.
  bridge_cmd(0x50, 1);
  junk.clear();
  drain(junk, 700);

  // 3. Auto-baud. Moduł dostraja się do bajtu 0x55 i odsyła swój adres.
  write_raw({kWake});
  junk.clear();
  drain(junk, 300);
  write_raw({kStop});             // gdyby został w trybie ciągłym po poprzednim uruchomieniu
  junk.clear();
  drain(junk, 200);
  tcflush(fd_, TCIFLUSH);
  rx_.clear();

  // 4. DOWÓD ŻYCIA. Bez tego stack wstawałby i sypał NaN-ami w nieskończoność,
  //    a przyczyna (nietrzymany PWREN, zły baud, rozjechane ramki) byłaby nie
  //    do odróżnienia jedna od drugiej. Numer seryjny nie odpala lasera, więc
  //    to tani test.
  std::vector<uint8_t> serial;
  if (!query_blocking(make_frame({0x80, 0x00, 0x0E}), kRegSerial, serial, 1200))
  {
    RCLCPP_ERROR(
      logger_,
      "Dalmierz JRT na %s @%d nie odpowiada. Sprawdź kolejno: PWREN (D4 mostka), "
      "baud modułu (u nas 38400, nie 19200), zasilanie 3,3 V modułu, "
      "TXD modułu -> D3 i RXD modułu <- D2.",
      device_port_.c_str(), module_baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string sn;
  char hex[4];
  for (const uint8_t b : serial)
  {
    std::snprintf(hex, sizeof(hex), "%02X", b);
    sn += hex;
  }
  RCLCPP_INFO(
    logger_, "Dalmierz JRT na %s @%d, nr seryjny %s, tryb %s, timeout %d ms",
    device_port_.c_str(), module_baud_, sn.c_str(), measure_mode_.c_str(), timeout_ms_);
  RCLCPP_INFO(logger_, "Zakres raportowany: %.2f-%.2f m, poza nim NaN", min_range_, max_range_);

  phase_ = Phase::Idle;
  have_clock_ = false;
  shots_ = 0;
  misses_ = 0;
  recoveries_ = 0;
  consecutive_misses_ = 0;
  recover_stage_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JrtLaserSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    write_raw({kStop});                                    // wyjdź z trybu ciągłego
    write_frame(make_frame({0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x00}));   // laser OFF
    std::vector<uint8_t> junk;
    drain(junk, 150);
    bridge_cmd(0x50, 0);                                   // PWREN w dół - moduł śpi
    drain(junk, 100);
    ::close(fd_);
    fd_ = -1;
  }
  RCLCPP_INFO(logger_, "Dalmierz JRT: %lu strzałów, %lu bez odpowiedzi, %lu odtworzeń łącza",
              static_cast<unsigned long>(shots_), static_cast<unsigned long>(misses_),
              static_cast<unsigned long>(recoveries_));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JrtLaserSensor::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (!have_clock_)
  {
    next_request_stamp_ = time;
    request_stamp_ = time;
    have_clock_ = true;
  }

  // --- zawsze dobieramy to, co przyszło: read() NIE CZEKA, tylko zgarnia ---
  uint8_t buf[256];
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0)
  {
    rx_.insert(rx_.end(), buf, buf + n);
  }

  if (phase_ == Phase::Waiting)
  {
    uint16_t reg = 0;
    bool is_error = false;
    std::vector<uint8_t> payload;
    while (extract_frame(reg, payload, is_error))
    {
      if (is_error)
      {
        // Ramka błędu: moduł zmierzył i wie, że się nie udało (cel poza
        // zasięgiem, sygnał za słaby...). To PEŁNOPRAWNY wynik pomiaru -
        // stemplujemy go, żeby broadcaster opublikował NaN zamiast milczeć.
        state_status_code_ = (payload.size() >= 2) ? ((payload[0] << 8) | payload[1]) : kNaN;
        state_range_ = kNaN;
        state_signal_quality_ = kNaN;
        state_sample_time_ = time.seconds();
        state_shot_start_ = request_stamp_.seconds();
        // Ramka błędu to DOWÓD ŻYCIA łącza - moduł odpowiedział, tylko nie miał
        // czego zmierzyć. Nie ma czego odtwarzać.
        consecutive_misses_ = 0;
        phase_ = Phase::Idle;
        next_request_stamp_ = time + rclcpp::Duration::from_seconds(period_ms_ / 1000.0);
        break;
      }
      if (reg == kRegResult && payload.size() >= 6)
      {
        const uint32_t mm = (static_cast<uint32_t>(payload[0]) << 24) |
                            (static_cast<uint32_t>(payload[1]) << 16) |
                            (static_cast<uint32_t>(payload[2]) << 8) |
                            static_cast<uint32_t>(payload[3]);
        const double range_m = static_cast<double>(mm) / 1000.0;

        state_signal_quality_ = (payload[4] << 8) | payload[5];
        state_status_code_ = 0.0;
        // Poza wiarygodnym zakresem raportujemy NaN, a nie liczbę: cichy błąd
        // przy rekonstrukcji chmury wygląda jak poprawny punkt.
        state_range_ = (range_m < min_range_ || range_m > max_range_) ? kNaN : range_m;
        state_sample_time_ = time.seconds();
        state_shot_start_ = request_stamp_.seconds();
        ++shots_;
        consecutive_misses_ = 0;
        phase_ = Phase::Idle;
        next_request_stamp_ = time + rclcpp::Duration::from_seconds(period_ms_ / 1000.0);
        break;
      }
      // Ramka na inny rejestr (np. spóźniona odpowiedź z konfiguracji) - pomiń.
    }

    if (phase_ == Phase::Waiting && (time - request_stamp_).seconds() * 1000.0 > timeout_ms_)
    {
      // Zgubiona odpowiedź nie może zawiesić czujnika na zawsze - stemplujemy
      // NaN i strzelamy od nowa.
      ++misses_;
      ++consecutive_misses_;
      state_range_ = kNaN;
      state_signal_quality_ = kNaN;
      state_status_code_ = kNaN;
      state_sample_time_ = time.seconds();
      state_shot_start_ = request_stamp_.seconds();

      if (consecutive_misses_ >= recover_after_misses_)
      {
        ++recoveries_;
        recover_stage_ = 0;
        recover_deadline_ = time;
        phase_ = Phase::Recovering;
        RCLCPP_WARN(
          logger_, "Dalmierz JRT: %d strzałów z rzędu bez odpowiedzi - odtwarzam łącze (%lu raz)",
          consecutive_misses_, static_cast<unsigned long>(recoveries_));
      }
      else
      {
        phase_ = Phase::Idle;
        next_request_stamp_ = time;
      }
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 5000,
        "Dalmierz JRT: brak odpowiedzi w %d ms (%lu z %lu strzałów)",
        timeout_ms_, static_cast<unsigned long>(misses_),
        static_cast<unsigned long>(shots_ + misses_));
    }
  }

  if (phase_ == Phase::Recovering)
  {
    if (step_recovery(time))
    {
      phase_ = Phase::Idle;
      next_request_stamp_ = time;
      RCLCPP_INFO(logger_, "Dalmierz JRT: łącze odtworzone, wracam do pomiarów");
    }
    return hardware_interface::return_type::OK;
  }

  if (phase_ == Phase::Idle && time >= next_request_stamp_)
  {
    rx_.clear();
    write_frame(measure_frame_);
    request_stamp_ = time;
    phase_ = Phase::Waiting;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::JrtLaserSensor, hardware_interface::SensorInterface)
