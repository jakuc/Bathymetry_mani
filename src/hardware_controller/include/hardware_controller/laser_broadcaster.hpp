#ifndef HARDWARE_CONTROLLER__LASER_BROADCASTER_HPP_
#define HARDWARE_CONTROLLER__LASER_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace hardware_controller
{

// Broadcaster dla LaserSensor. Robi to samo co gotowy range_sensor_broadcaster
// (sensor_msgs/msg/Range z state_interface "<sensor_name>/range"), ale z dwiema
// różnicami, bez których rekonstrukcja chmury jest gorsza:
//
//  1. PUBLIKUJE TYLKO ŚWIEŻE POMIARY. Pętla ros2_control kręci się 50 Hz,
//     a dalmierz daje realnie ~18,6 Hz - gotowy broadcaster republikowałby ten
//     sam odczyt średnio 2,7 raza, a kolektor nie miałby jak odróżnić duplikatu
//     od dwóch pomiarów o tej samej wartości. Tu odczyt idzie na topic dokładnie
//     raz, bo bramkuje go rosnący "<sensor_name>/sample_time".
//  2. ZNACZNIK CZASU TO CZAS POMIARU, nie czas publikacji. Przy sweepie głowica
//     się rusza, więc TF trzeba odpytać o chwilę, w której padł promień, a nie
//     o chwilę, w której wiadomość wyszła z kontrolera.
//
// Obok "~/range" publikuje "~/raw" - surowy odczyt ADC, z DOKŁADNIE tym samym
// znacznikiem czasu co odpowiadający mu Range. Bez tego CSV skanu niesie tylko
// metry, czyli skutek jednej wybranej kalibracji, i sprawdzenie innej krzywej
// wymaga skanowania od nowa. Ze stemplem jako kluczem parowania jeden skan
// wystarcza na dowolnie wiele wariantów przeliczenia offline.
//
// Typ Vector3Stamped jest użyty umownie - własna wiadomość kosztowałaby
// przebudowę paczki interfejsów na Pi 3B, a potrzebny jest tylko stempel plus
// kilka liczb. Znaczenie pól (i to jest jedyne miejsce, gdzie jest zapisane):
//     x = adc          - mediana surowego odczytu ADC
//     y = adc_spread   - rozrzut (max-min) w oknie medianowym
//     z = sample_time  - stempel (dla JRT: moment ROZPOCZĘCIA strzału)
//     z = sample_time  - czas pomiaru w sekundach, ten sam co w stemplu
class LaserBroadcaster : public controller_interface::ControllerInterface
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
  std::string sensor_name_;
  std::string frame_id_;
  double field_of_view_{0.0};
  double min_range_{1.0};
  double max_range_{5.5};
  int radiation_type_{1};  // sensor_msgs::msg::Range::INFRARED

  // Osobno dla każdego topiku - patrz komentarz przy publikacji raw w update().
  double last_sample_time_{0.0};
  double last_raw_sample_time_{0.0};

  // Nazwy interfejsów diagnostycznych trafiających na ~/raw (x, y).
  std::string diag0_{"adc"};
  std::string diag1_{"adc_spread"};
  std::string diag2_{"sample_time"};

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Range>> rt_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr raw_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> rt_raw_publisher_;
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__LASER_BROADCASTER_HPP_
