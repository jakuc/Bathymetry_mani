#include "hardware_controller/gnss_sensor.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <limits>

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
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 115200: return B115200;
    default: return B115200;
  }
}

// Puste pole NMEA → NaN (mosaic streamuje ramki z pustymi polami zanim złapie fixa).
double field_to_double(const std::string & field)
{
  return field.empty() ? kNaN : std::stod(field);
}

// NMEA koduje współrzędne jako (d)ddmm.mmmmm — zamiana na stopnie dziesiętne.
double nmea_coord_to_deg(const std::string & field, const std::string & hemisphere)
{
  if (field.empty())
  {
    return kNaN;
  }
  const double raw = std::stod(field);
  const double degrees = std::floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  const double result = degrees + minutes / 60.0;
  return (hemisphere == "S" || hemisphere == "W") ? -result : result;
}

// Kody z bloku NTRIPClientStatus (mosaic-H Reference Guide, sekcja 4).
const char * ntrip_status_text(int status)
{
  switch (status)
  {
    case 0: return "połączenie wyłączone";
    case 1: return "inicjalizacja";
    case 2: return "działa — korekcje płyną";
    case 3: return "błąd";
    case 4: return "ponawianie połączenia";
    case 5: return "wyłączone (duplikat innego połączenia NTRIP)";
    default: return "nieznany status";
  }
}

const char * ntrip_error_text(int error)
{
  switch (error)
  {
    case 0: return "brak błędu";
    case 1: return "błąd inicjalizacji (np. nie pobrano source table)";
    case 2: return "błąd autoryzacji — zły login/hasło";
    case 3: return "błąd połączenia — brak internetu po stronie odbiornika?";
    case 4: return "mountpoint nie istnieje";
    case 5: return "mountpoint niedostępny";
    case 6: return "caster czeka na GGA z odbiornika";
    case 7: return "wysyłanie GGA wyłączone, a mountpoint go wymaga";
    case 8: return "nie rozwiązano nazwy hosta (DNS)";
    case 9: return "poza obszarem obsługi serwisu";
    case 10: return "błąd konfiguracji TLS";
    case 11: return "błąd handshake TLS";
    case 12: return "błąd odcisku certyfikatu TLS";
    case 13: return "nieznany czas — nie da się zweryfikować certyfikatu TLS";
    case 254: return "nieznany błąd";
    default: return "nieudokumentowany kod błędu";
  }
}
}  // namespace

hardware_interface::CallbackReturn GnssSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters.at("device_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  receiver_port_ = info_.hardware_parameters.at("receiver_port");
  nmea_stream_ = std::stoi(info_.hardware_parameters.at("nmea_stream"));
  nmea_rate_ = info_.hardware_parameters.at("nmea_rate");
  sensor_name_ = info_.sensors.at(0).name;

  // Parametry RTK są opcjonalne — starsze URDF-y ich nie mają, a brak wpisu
  // ma znaczyć "zostaw domyślne", nie "wywal się w on_init".
  const auto param_or = [this](const char * name, const std::string & fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };
  status_stream_ = std::stoi(param_or("status_stream", std::to_string(status_stream_)));
  status_rate_ = param_or("status_rate", status_rate_);
  ntrip_connection_ = std::stoi(param_or("ntrip_connection", std::to_string(ntrip_connection_)));

  state_latitude_ = kNaN;
  state_longitude_ = kNaN;
  state_altitude_ = kNaN;
  state_fix_quality_ = 0.0;
  state_num_satellites_ = 0.0;
  state_lat_std_ = kNaN;
  state_lon_std_ = kNaN;
  state_alt_std_ = kNaN;
  state_heading_ = kNaN;
  state_diff_age_ = kNaN;
  state_station_id_ = kNaN;
  state_ntrip_status_ = kNaN;
  state_ntrip_error_ = kNaN;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GnssSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(sensor_name_, "latitude", &state_latitude_);
  interfaces.emplace_back(sensor_name_, "longitude", &state_longitude_);
  interfaces.emplace_back(sensor_name_, "altitude", &state_altitude_);
  interfaces.emplace_back(sensor_name_, "fix_quality", &state_fix_quality_);
  interfaces.emplace_back(sensor_name_, "num_satellites", &state_num_satellites_);
  interfaces.emplace_back(sensor_name_, "lat_std", &state_lat_std_);
  interfaces.emplace_back(sensor_name_, "lon_std", &state_lon_std_);
  interfaces.emplace_back(sensor_name_, "alt_std", &state_alt_std_);
  interfaces.emplace_back(sensor_name_, "heading", &state_heading_);
  interfaces.emplace_back(sensor_name_, "diff_age", &state_diff_age_);
  interfaces.emplace_back(sensor_name_, "station_id", &state_station_id_);
  interfaces.emplace_back(sensor_name_, "ntrip_status", &state_ntrip_status_);
  interfaces.emplace_back(sensor_name_, "ntrip_error", &state_ntrip_error_);
  interfaces.emplace_back(sensor_name_, "data_count", &state_data_count_);
  return interfaces;
}

hardware_interface::CallbackReturn GnssSensor::on_configure(const rclcpp_lifecycle::State &)
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

  // Automatyczna konfiguracja odbiornika: włącz strumień NMEA na porcie, z
  // którego czytamy. Zweryfikowane na żywo na mosaic-H: port przyjmuje komendy
  // i NMEA na tym samym łączu, odpowiedzi ("$R: ...", prompt "USB1>") są
  // ignorowane przez parser, bo nie kończą się tagiem GGA/GST/HDT.
  send_receiver_command(
    "setNMEAOutput, Stream" + std::to_string(nmea_stream_) + ", " + receiver_port_ +
    ", GGA+GST+HDT, " + nmea_rate_);
  RCLCPP_INFO(
    logger_, "Skonfigurowano %s: NMEA GGA+GST+HDT @ %s na %s",
    device_port_.c_str(), nmea_rate_.c_str(), receiver_port_.c_str());

  // Status klienta NTRIP osobnym, wolniejszym strumieniem. Samo połączenie z
  // casterem konfiguruje się raz na stałe (scripts/gnss_rtk.py) i siedzi w
  // pamięci nieulotnej odbiornika — tutaj tylko je OBSERWUJEMY, żeby nie
  // trzymać w URDF-ie loginu i hasła do serwisu RTK.
  if (status_stream_ > 0)
  {
    send_receiver_command(
      "setNMEAOutput, Stream" + std::to_string(status_stream_) + ", " + receiver_port_ +
      ", SNC, " + status_rate_);
    RCLCPP_INFO(
      logger_, "Status NTRIP: $PSSN,SNC @ %s dla NTR%d", status_rate_.c_str(), ntrip_connection_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GnssSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    // Zostaw odbiornik czysty — wyłącz nasze strumienie NMEA. (Ustawienie
    // deskryptora połączenia na "none" to udokumentowany sposób wyłączenia
    // strumienia; połączenia NTRIP to NIE dotyczy — ma zostać włączone.)
    send_receiver_command("setNMEAOutput, Stream" + std::to_string(nmea_stream_) + ", none");
    if (status_stream_ > 0)
    {
      send_receiver_command("setNMEAOutput, Stream" + std::to_string(status_stream_) + ", none");
    }
    ::close(fd_);
    fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void GnssSensor::send_receiver_command(const std::string & command)
{
  const std::string framed = command + "\r\n";
  if (::write(fd_, framed.c_str(), framed.size()) != static_cast<ssize_t>(framed.size()))
  {
    RCLCPP_WARN(logger_, "Nie udało się wysłać komendy do odbiornika: '%s'", command.c_str());
  }
  tcdrain(fd_);
  wait_for_command_ack(command);
}

// Odbiornik potwierdza komendę echem "$R: <komenda>" zakończonym promptem.
// Bez odczekania na to potwierdzenie kolejna komenda wysłana od razu POTRAFI
// PRZEPAŚĆ — przepalone na żywo: setNMEAOutput dla strumienia statusu NTRIP
// ginęło bez śladu, bo poprzednia komenda właśnie uruchamiała strumień 10 Hz
// na tym samym porcie. Odczytane tu bajty wyrzucamy: to faza konfiguracji,
// żaden stan jeszcze nie jest publikowany.
void GnssSensor::wait_for_command_ack(const std::string & command)
{
  char buf[512];
  std::string reply;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);

  while (std::chrono::steady_clock::now() < deadline)
  {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0)
    {
      reply.append(buf, static_cast<size_t>(n));
      const size_t echo = reply.find("$R:");
      // Prompt ("USB1>") po echu komendy = odbiornik skończył ją przetwarzać.
      if (echo != std::string::npos && reply.find('>', echo) != std::string::npos)
      {
        return;
      }
    }
    else
    {
      usleep(5000);
    }
  }

  RCLCPP_WARN(
    logger_, "Odbiornik nie potwierdził komendy w 1.5 s: '%s'", command.c_str());
}

// Stan NTRIP zmienia się rzadko (przy zestawianiu łącza i przy awarii), więc
// logujemy przejścia, a nie okresowy stan. To jedyny komunikat, który w terenie
// odróżnia "zły mountpoint" od "brak internetu" bez wchodzenia do web UI.
void GnssSensor::log_ntrip_transition()
{
  const int status = static_cast<int>(state_ntrip_status_);
  const int error = static_cast<int>(state_ntrip_error_);
  if (status == last_logged_ntrip_status_ && error == last_logged_ntrip_error_)
  {
    return;
  }
  last_logged_ntrip_status_ = status;
  last_logged_ntrip_error_ = error;

  if (status == 2)
  {
    RCLCPP_INFO(logger_, "NTR%d: %s", ntrip_connection_, ntrip_status_text(status));
  }
  else if (status == 3 || status == 4)
  {
    RCLCPP_WARN(
      logger_, "NTR%d: %s — %s", ntrip_connection_, ntrip_status_text(status),
      ntrip_error_text(error));
  }
  else
  {
    RCLCPP_INFO(logger_, "NTR%d: %s", ntrip_connection_, ntrip_status_text(status));
  }
}

hardware_interface::return_type GnssSensor::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  poll_serial();
  return hardware_interface::return_type::OK;
}

void GnssSensor::poll_serial()
{
  char buf[512];
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0)
  {
    line_buffer_.append(buf, static_cast<size_t>(n));
  }

  // Przy odpiętym USB read() zwraca błąd (EIO/ENODEV), nie EAGAIN — bez tego
  // logu urządzenie znika po cichu, a stany zamarzają na ostatnich wartościach.
  if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000,
      "Błąd odczytu z %s (errno %d) — urządzenie odpięte?", device_port_.c_str(), errno);
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

void GnssSensor::process_line(const std::string & line)
{
  // Prompt odbiornika ("USB1>") nie jest zakończony nowym wierszem i skleja
  // się z następnym zdaniem NMEA — tnij wszystko przed ostatnim '$'.
  const size_t dollar = line.rfind('$');
  if (dollar == std::string::npos)
  {
    return;
  }
  const size_t star = line.find('*', dollar);
  std::string body =
    line.substr(dollar, star == std::string::npos ? std::string::npos : star - dollar);

  // Zdania proprietary Septentrio (tu: SNC) grupują podbloki w nawiasach
  // kwadratowych: "$PSSN,SNC,[0,tow,wnc,[1,2,0,0]]". Pola i tak są rozdzielone
  // przecinkami, więc wystarczy zdjąć nawiasy i czytać płaską listę.
  if (body.compare(0, 5, "$PSSN") == 0)
  {
    body.erase(std::remove_if(body.begin(), body.end(), [](char c) {
      return c == '[' || c == ']';
    }), body.end());
  }

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

  const std::string & tag = fields[0];
  auto tag_is = [&tag](const char * suffix) {
    return tag.size() >= 3 && tag.compare(tag.size() - 3, 3, suffix) == 0;
  };

  try
  {
    if (tag_is("GGA") && fields.size() > 11)
    {
      state_latitude_ = nmea_coord_to_deg(fields[2], fields[3]);
      state_longitude_ = nmea_coord_to_deg(fields[4], fields[5]);
      state_fix_quality_ = fields[6].empty() ? 0.0 : std::stod(fields[6]);
      state_num_satellites_ = fields[7].empty() ? 0.0 : std::stod(fields[7]);
      // NavSatFix oczekuje wysokości elipsoidalnej; GGA daje MSL (pole 9)
      // i separację geoidy (pole 11) — elipsoidalna to ich suma.
      const double msl = field_to_double(fields[9]);
      const double geoid_sep = field_to_double(fields[11]);
      state_altitude_ = msl + geoid_sep;
      // Pola 13/14 (wiek korekcji, ID stacji) są puste bez RTK/DGPS — wtedy
      // field_to_double daje NaN, co jest poprawną odpowiedzią "brak korekcji".
      state_diff_age_ = fields.size() > 13 ? field_to_double(fields[13]) : kNaN;
      state_station_id_ = fields.size() > 14 ? field_to_double(fields[14]) : kNaN;
      state_data_count_ += 1.0;
    }
    else if (tag_is("GST") && fields.size() > 8)
    {
      state_lat_std_ = field_to_double(fields[6]);
      state_lon_std_ = field_to_double(fields[7]);
      state_alt_std_ = field_to_double(fields[8]);
    }
    else if (tag_is("HDT") && fields.size() > 1)
    {
      state_heading_ = field_to_double(fields[1]);
    }
    else if (tag == "$PSSN" && fields.size() > 1 && fields[1] == "SNC")
    {
      // Po zdjęciu nawiasów: $PSSN,SNC,rewizja,TOW,WNc, potem po 4 pola na
      // każde połączenie NTRIP: CDIndex,Status,ErrorCode,Info.
      for (size_t i = 5; i + 3 < fields.size(); i += 4)
      {
        if (fields[i].empty() || std::stoi(fields[i]) != ntrip_connection_)
        {
          continue;
        }
        state_ntrip_status_ = field_to_double(fields[i + 1]);
        state_ntrip_error_ = field_to_double(fields[i + 2]);
        log_ntrip_transition();
        break;
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger_, "Nie udało się sparsować zdania NMEA '%s': %s", line.c_str(), e.what());
  }
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::GnssSensor, hardware_interface::SensorInterface)
