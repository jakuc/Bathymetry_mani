#include "hardware_controller/gnss_broadcaster.hpp"

#include <cmath>
#include <cstdio>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

namespace
{
// Wskaźnik jakości z GGA. RTK fixed (4) vs float (5) to różnica rzędu
// centymetry vs decymetry — NavSatStatus nie ma na to osobnych stopni
// (oba lądują w GBAS_FIX), więc rozróżnienie idzie tylko przez diagnostykę.
const char * fix_quality_text(int quality)
{
  switch (quality)
  {
    case 0: return "brak fixa";
    case 1: return "single-point (bez korekcji)";
    case 2: return "DGPS/SBAS";
    case 4: return "RTK fixed";
    case 5: return "RTK float";
    case 6: return "dead reckoning";
    default: return "inny";
  }
}

const char * ntrip_status_text(int status)
{
  switch (status)
  {
    case 0: return "wyłączony";
    case 1: return "inicjalizacja";
    case 2: return "działa";
    case 3: return "błąd";
    case 4: return "ponawianie";
    case 5: return "duplikat";
    default: return "brak danych";
  }
}

const char * ntrip_error_text(int error)
{
  switch (error)
  {
    case 0: return "brak błędu";
    case 1: return "błąd inicjalizacji";
    case 2: return "błąd autoryzacji";
    case 3: return "błąd połączenia";
    case 4: return "mountpoint nie istnieje";
    case 5: return "mountpoint niedostępny";
    case 6: return "czeka na GGA";
    case 7: return "GGA wyłączone, wymagane przez mountpoint";
    case 8: return "błąd DNS";
    case 9: return "poza obszarem serwisu";
    case 10: return "błąd konfiguracji TLS";
    case 11: return "błąd handshake TLS";
    case 12: return "błąd odcisku TLS";
    case 13: return "nieznany czas (TLS)";
    case 254: return "nieznany błąd";
    default: return "brak danych";
  }
}

std::string fmt(double value, int precision = 2)
{
  if (!std::isfinite(value))
  {
    return "—";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", precision, value);
  return std::string(buf);
}
}  // namespace

controller_interface::CallbackReturn GnssBroadcaster::on_init()
{
  try
  {
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("frame_id", "");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Wyjątek w on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GnssBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration GnssBroadcaster::state_interface_configuration() const
{
  // Kolejność tutaj = kolejność w state_interfaces_ w update().
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name_ + "/latitude",
      sensor_name_ + "/longitude",
      sensor_name_ + "/altitude",
      sensor_name_ + "/fix_quality",
      sensor_name_ + "/lat_std",
      sensor_name_ + "/lon_std",
      sensor_name_ + "/alt_std",
      sensor_name_ + "/heading",
      sensor_name_ + "/data_count",
      sensor_name_ + "/num_satellites",
      sensor_name_ + "/diff_age",
      sensor_name_ + "/station_id",
      sensor_name_ + "/ntrip_status",
      sensor_name_ + "/ntrip_error",
    }};
}

controller_interface::CallbackReturn GnssBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();

  if (sensor_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (frame_id_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  fix_publisher_ = get_node()->create_publisher<sensor_msgs::msg::NavSatFix>(
    "~/fix", rclcpp::SystemDefaultsQoS());
  rt_fix_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::NavSatFix>>(fix_publisher_);

  heading_publisher_ = get_node()->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "~/heading", rclcpp::SystemDefaultsQoS());
  rt_heading_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::QuaternionStamped>>(heading_publisher_);

  rt_fix_publisher_->lock();
  rt_fix_publisher_->msg_.header.frame_id = frame_id_;
  // mosaic-H śledzi wszystkie konstelacje.
  rt_fix_publisher_->msg_.status.service =
    sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
    sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  rt_fix_publisher_->unlock();

  rt_heading_publisher_->lock();
  rt_heading_publisher_->msg_.header.frame_id = frame_id_;
  rt_heading_publisher_->unlock();

  // Diagnostyka RTK na standardowym /diagnostics: jedno `ros2 topic echo`
  // odpowiada w terenie na pytanie "dlaczego nie ma RTK" — jakość fixa,
  // wiek korekcji i status klienta NTRIP naraz.
  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  rt_diagnostics_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
      diagnostics_publisher_);

  // Klucze są stałe — alokujemy je raz, w update() podmieniamy tylko wartości.
  rt_diagnostics_publisher_->lock();
  auto & diag = rt_diagnostics_publisher_->msg_;
  diag.status.resize(1);
  diag.status[0].name = "gnss: RTK";
  diag.status[0].hardware_id = sensor_name_;
  diag.status[0].values.resize(kDiagKeyCount);
  diag.status[0].values[0].key = "jakość fixa";
  diag.status[0].values[1].key = "satelity";
  diag.status[0].values[2].key = "wiek korekcji [s]";
  diag.status[0].values[3].key = "stacja bazowa / VRS";
  diag.status[0].values[4].key = "klient NTRIP";
  diag.status[0].values[5].key = "std E/N/U [m]";
  rt_diagnostics_publisher_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GnssBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GnssBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GnssBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const double latitude = state_interfaces_[0].get_value();
  const double longitude = state_interfaces_[1].get_value();
  const double altitude = state_interfaces_[2].get_value();
  const double fix_quality = state_interfaces_[3].get_value();
  const double lat_std = state_interfaces_[4].get_value();
  const double lon_std = state_interfaces_[5].get_value();
  const double alt_std = state_interfaces_[6].get_value();
  const double heading_deg = state_interfaces_[7].get_value();
  const double data_count = state_interfaces_[8].get_value();
  const double num_satellites = state_interfaces_[9].get_value();
  const double diff_age = state_interfaces_[10].get_value();
  const double station_id = state_interfaces_[11].get_value();
  const double ntrip_status = state_interfaces_[12].get_value();
  const double ntrip_error = state_interfaces_[13].get_value();

  // Diagnostyka leci PRZED bramką świeżości i własnym, wolnym tempem: ma
  // raportować także sytuację "odbiornik zamilkł", w której /gnss/fix z
  // założenia nic nie publikuje.
  // (Sekundy jako double, nie rclcpp::Time — odejmowanie Time o różnych
  // źródłach czasu rzuca wyjątkiem, a domyślnie skonstruowany Time ma inne
  // źródło niż zegar węzła.)
  if (time.seconds() - last_diagnostics_time_ >= kDiagnosticsPeriod)
  {
    publish_diagnostics(
      time, fix_quality, num_satellites, diff_age, station_id, ntrip_status, ntrip_error,
      lat_std, lon_std, alt_std, data_count);
    last_diagnostics_time_ = time.seconds();
  }

  // Publikuj tylko świeże ramki — patrz komentarz w nagłówku.
  if (data_count == last_data_count_)
  {
    return controller_interface::return_type::OK;
  }
  last_data_count_ = data_count;

  if (rt_fix_publisher_ && rt_fix_publisher_->trylock())
  {
    auto & msg = rt_fix_publisher_->msg_;
    msg.header.stamp = time;
    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;

    // Mapowanie jakości fixa GGA na NavSatStatus: 0 → brak fixa,
    // 2 (DGPS/SBAS) → SBAS_FIX, 4/5 (RTK fix/float) → GBAS_FIX (najbliższy
    // dostępny stopień "fix z augmentacją naziemną"), reszta → FIX.
    if (fix_quality < 0.5)
    {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    else if (fix_quality > 3.5 && fix_quality < 5.5)
    {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    }
    else if (fix_quality > 1.5 && fix_quality < 2.5)
    {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    }
    else
    {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }

    // NavSatFix: kowariancja w ENU — [0]=wschód (lon), [4]=północ (lat), [8]=góra.
    if (std::isfinite(lat_std) && std::isfinite(lon_std) && std::isfinite(alt_std))
    {
      msg.position_covariance.fill(0.0);
      msg.position_covariance[0] = lon_std * lon_std;
      msg.position_covariance[4] = lat_std * lat_std;
      msg.position_covariance[8] = alt_std * alt_std;
      msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    else
    {
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    rt_fix_publisher_->unlockAndPublish();
  }

  // Heading tylko gdy odbiornik ma rozwiązanie attitude z dwóch anten.
  if (std::isfinite(heading_deg) && rt_heading_publisher_ && rt_heading_publisher_->trylock())
  {
    // NMEA HDT: stopnie od północy, zgodnie z ruchem wskazówek zegara.
    // ROS (ENU): yaw w radianach od osi wschodu, przeciwnie do wskazówek.
    const double yaw_enu = (90.0 - heading_deg) * M_PI / 180.0;
    auto & msg = rt_heading_publisher_->msg_;
    msg.header.stamp = time;
    msg.quaternion.x = 0.0;
    msg.quaternion.y = 0.0;
    msg.quaternion.z = std::sin(yaw_enu / 2.0);
    msg.quaternion.w = std::cos(yaw_enu / 2.0);
    rt_heading_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void GnssBroadcaster::publish_diagnostics(
  const rclcpp::Time & time, double fix_quality, double num_satellites, double diff_age,
  double station_id, double ntrip_status, double ntrip_error, double lat_std, double lon_std,
  double alt_std, double data_count)
{
  if (!rt_diagnostics_publisher_ || !rt_diagnostics_publisher_->trylock())
  {
    return;
  }

  const bool stale = (data_count == last_diagnostics_data_count_);
  last_diagnostics_data_count_ = data_count;

  const int quality = static_cast<int>(fix_quality);
  const int ntrip = std::isfinite(ntrip_status) ? static_cast<int>(ntrip_status) : -1;

  auto & status = rt_diagnostics_publisher_->msg_.status[0];
  if (stale)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "brak ramek z odbiornika — urządzenie odpięte?";
  }
  else if (quality == 4)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "RTK fixed";
  }
  else if (quality == 0)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "brak fixa";
  }
  else
  {
    // Wszystko poza RTK fixed to dla batymetrii degradacja dokładności —
    // stąd WARN nawet przy poprawnym fixie single-point.
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = std::string("bez RTK fixed: ") + fix_quality_text(quality);
  }

  status.values[0].value = std::string(fix_quality_text(quality)) + " (" + fmt(fix_quality, 0) + ")";
  status.values[1].value = fmt(num_satellites, 0);
  status.values[2].value = fmt(diff_age, 1);
  status.values[3].value = fmt(station_id, 0);
  status.values[4].value = ntrip < 0
                             ? std::string("brak zdania SNC (strumień wyłączony?)")
                             : std::string(ntrip_status_text(ntrip)) + " / " +
                                 ntrip_error_text(static_cast<int>(ntrip_error));
  status.values[5].value = fmt(lon_std) + " / " + fmt(lat_std) + " / " + fmt(alt_std);

  rt_diagnostics_publisher_->msg_.header.stamp = time;
  rt_diagnostics_publisher_->unlockAndPublish();
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::GnssBroadcaster, controller_interface::ControllerInterface)
