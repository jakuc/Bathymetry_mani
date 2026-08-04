#ifndef HARDWARE_CONTROLLER__GNSS_BROADCASTER_HPP_
#define HARDWARE_CONTROLLER__GNSS_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace hardware_controller
{

// Broadcaster dla stanów GnssSensor — ros2_controllers w Humble nie ma
// gotowego brokera NavSatFix (gps_sensor_broadcaster pojawił się w późniejszych
// dystrybucjach). Publikuje:
//   ~/fix     (sensor_msgs/NavSatFix)          — pozycja + kowariancja z GST
//   ~/heading (geometry_msgs/QuaternionStamped) — yaw ENU z headingu dwuantenowego,
//              publikowany tylko gdy odbiornik ma rozwiązanie attitude (nie-NaN)
//   /diagnostics (diagnostic_msgs/DiagnosticArray) — stan RTK raz na sekundę:
//              jakość fixa (fixed vs float — NavSatStatus tego nie rozróżnia),
//              wiek korekcji, stacja bazowa, status klienta NTRIP
//
// Publikuje wyłącznie gdy GnssSensor odebrał nową ramkę (stan "data_count"
// się zmienił) — topiki chodzą w rytmie prawdziwych danych z odbiornika
// (~10 Hz przy msec100), a po odpięciu urządzenia MILKNĄ, zamiast powtarzać
// zamrożone wartości w takcie update_rate.
class GnssBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publish_diagnostics(
    const rclcpp::Time & time, double fix_quality, double num_satellites, double diff_age,
    double station_id, double ntrip_status, double ntrip_error, double lat_std, double lon_std,
    double alt_std, double data_count);

  static constexpr size_t kDiagKeyCount = 6;
  static constexpr double kDiagnosticsPeriod = 1.0;  // s

  std::string sensor_name_;
  std::string frame_id_;
  double last_data_count_{-1.0};
  double last_diagnostics_time_{-1.0e9};
  double last_diagnostics_data_count_{-1.0};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::NavSatFix>> rt_fix_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::QuaternionStamped>> rt_heading_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> rt_diagnostics_publisher_;
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__GNSS_BROADCASTER_HPP_
