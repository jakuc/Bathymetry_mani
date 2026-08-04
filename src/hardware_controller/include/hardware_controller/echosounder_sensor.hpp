#ifndef HARDWARE_CONTROLLER__ECHOSOUNDER_SENSOR_HPP_
#define HARDWARE_CONTROLLER__ECHOSOUNDER_SENSOR_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_controller
{

// Echosonda DeepVision/Satlab SLD-100: NMEA 0183 po USB-serial, 4800 8N1.
// Zdania potwierdzone na żywym sprzęcie: $SDDPT (głębokość, puste pola gdy
// brak echa) i $SDMTW (temperatura wody). Checksum nie jest weryfikowany —
// tak samo jak w potwierdzonym teście na hoście (pyserial read), urządzenie
// nie generuje uszkodzonych ramek na tyle często by to było potrzebne.
class EchosounderSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void process_line(const std::string & line);
  void poll_serial();

  std::string sensor_name_;
  std::string device_port_;
  int baud_rate_{4800};
  int fd_{-1};

  std::string line_buffer_;
  double state_range_{0.0};
  double state_water_temperature_{0.0};

  rclcpp::Logger logger_{rclcpp::get_logger("EchosounderSensor")};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__ECHOSOUNDER_SENSOR_HPP_
