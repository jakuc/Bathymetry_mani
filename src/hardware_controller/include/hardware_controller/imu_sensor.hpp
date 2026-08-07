#ifndef HARDWARE_CONTROLLER__IMU_SENSOR_HPP_
#define HARDWARE_CONTROLLER__IMU_SENSOR_HPP_

#include <cstdint>
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

// GY-955 — to jest BNO055, ale z własnym MCU i wyjściem UART. UART obchodzi
// znany bug clock stretchingu I2C w BCM2837, przez który BNO055 po I2C na Pi
// nie działa. To NIE jest natywny protokół BNO055 (na `AA 01 00 01` moduł nie
// odpowiada) — MCU płytki ma własny format ramki, opisany niżej.
//
// Na płytce bathset moduł wisi na /dev/serial0 (ttyS0), 9600 8N1.
// Protokół i skale zweryfikowane na żywym module 2026-07-28.
struct ImuProtocol
{
  static constexpr uint8_t kHeaderByte = 0x5A;   // ramka zaczyna się od 5A 5A
  static constexpr uint8_t kConfigPrefix = 0xAA; // konfiguracja TRWAŁA (zapis w module)
  static constexpr uint8_t kQueryPrefix = 0xA5;  // zapytanie jednorazowe

  // Bitmaska ZAWARTOŚCI ramki (bajt 2). To NIE jest typ ramki — pomylenie tego
  // kończy się dekodowaniem akcelerometru jako kątów Eulera, co daje wartości
  // wyglądające całkiem wiarygodnie.
  static constexpr uint8_t kMaskAcc = 0x01;
  static constexpr uint8_t kMaskMag = 0x02;
  static constexpr uint8_t kMaskGyr = 0x04;
  static constexpr uint8_t kMaskEuler = 0x08;
  static constexpr uint8_t kMaskQuat = 0x10;

  // Rozmiary bloków danych [B]; każda oś to int16 big-endian.
  static constexpr int kSizeAcc = 6;
  static constexpr int kSizeMag = 6;
  static constexpr int kSizeGyr = 6;
  static constexpr int kSizeEuler = 6;
  static constexpr int kSizeQuat = 8;

  // Skale surowych wartości (LSB na jednostkę).
  static constexpr double kAccPerMs2 = 100.0;   // m/s^2
  static constexpr double kMagPerUt = 16.0;     // uT
  static constexpr double kGyrPerDps = 16.0;    // deg/s
  static constexpr double kEulerPerDeg = 100.0; // deg
  static constexpr double kQuatPerUnit = 10000.0;
};

class ImuSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void poll_serial();
  // Próbuje wyciąć i zdekodować jedną ramkę z początku bufora.
  // Zwraca liczbę bajtów do skonsumowania (0 = za mało danych, czekamy).
  size_t try_parse_frame();
  int expected_data_size(uint8_t mask) const;
  void decode(uint8_t mask, const uint8_t * data, uint8_t calib);
  void send_config();

  std::string sensor_name_;
  std::string device_port_;
  int baud_rate_{9600};
  int fd_{-1};

  // Bajt konfiguracyjny wysyłany przy starcie (patrz on_configure).
  uint8_t module_cmd_{0xB5};
  bool configure_module_{true};

  // Kolejność składowych kwaternionu w ramce. BNO055 natywnie podaje W,X,Y,Z
  // i moduł najpewniej to powiela, ale instrukcja GY-955 tego nie precyzuje,
  // więc trzymamy to jako parametr - gdyby orientacja wychodziła przekręcona,
  // przełącza się z linii poleceń zamiast rekompilować.
  bool quat_wxyz_{true};

  std::vector<uint8_t> buffer_;

  // Stan wystawiany na interfejsy. NaN znaczy "moduł tego nie przysyła" —
  // sensor_msgs/Imu ma na to konwencję (kowariancja[0] = -1), a zero udawałoby
  // realny pomiar.
  double orientation_[4]{0.0, 0.0, 0.0, 1.0};   // x, y, z, w
  double angular_velocity_[3]{0.0, 0.0, 0.0};   // rad/s
  double linear_acceleration_[3]{0.0, 0.0, 0.0};// m/s^2
  // Status kalibracji z bajtu SGAM, układ jak CALIB_STAT w BNO055:
  // bity 7:6 SYS, 5:4 GYR, 3:2 ACC, 1:0 MAG, każdy 0-3.
  double calib_sys_{0.0};
  double calib_gyr_{0.0};
  double calib_acc_{0.0};
  double calib_mag_{0.0};

  uint64_t frames_ok_{0};
  uint64_t frames_bad_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("ImuSensor")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__IMU_SENSOR_HPP_
