#ifndef HARDWARE_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_
#define HARDWARE_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_controller
{

// Adresy rejestrów i stałe kalibracji przeniesione 1:1 z
// xm540_bringup/xm540_bringup/config.py (ten plik pozostaje nietknięty —
// to jest port istniejącej, sprawdzonej konfiguracji na C++, nie nowa kalibracja).
struct DynamixelRegisters
{
  static constexpr uint8_t ADDR_TORQUE_ENABLE = 64;
  static constexpr uint8_t ADDR_OPERATING_MODE = 11;
  static constexpr uint16_t ADDR_GOAL_POSITION = 116;
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
  static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
  static constexpr uint16_t ADDR_PRESENT_CURRENT = 126;
  static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;
  static constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
  static constexpr uint16_t ADDR_PROFILE_ACCELERATION = 108;
  static constexpr uint8_t MODE_POSITION = 3;
};

struct JointHandle
{
  std::string name;
  uint8_t servo_id{0};
  // Pozycja serwa (surowa) odpowiadająca zeru jointa w URDF-ie. Domyślnie środek
  // zakresu enkodera, ale po zmontowaniu konstrukcji zero mechaniczne prawie nigdy
  // nie wypada w środku - stąd parametr per joint, a nie wspólna stała.
  int32_t center_raw{2048};
  double command_position{0.0};   // rad, zadana pozycja (interfejs komend)
  double state_position{0.0};     // rad
  double state_velocity{0.0};     // surowa jednostka serwa (bez konwersji — jak w dynamixel_node.py)
  double state_effort{0.0};       // surowy prąd (bez konwersji — jak w dynamixel_node.py)
  double state_temperature{0.0};  // °C
};

class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double raw_to_rad(int32_t raw, int32_t center_raw) const;
  int32_t rad_to_raw(double rad, int32_t center_raw) const;

  void write1(uint8_t servo_id, uint16_t addr, uint8_t value);
  void write4(uint8_t servo_id, uint16_t addr, int32_t value);
  uint8_t read1(uint8_t servo_id, uint16_t addr);
  uint16_t read2(uint8_t servo_id, uint16_t addr);
  int32_t read4(uint8_t servo_id, uint16_t addr);

  std::string device_port_;
  int baud_rate_{1000000};
  // Profil ruchu serwa. UWAGA: 0 u Dynamixela nie znaczy "zero prędkości", tylko
  // "bez profilu" — serwo skacze do zadanej pozycji z maksymalną prędkością.
  // Dla samego manipulatora to było nieszkodliwe, ale z zamontowaną głowicą
  // każdy skok komendy jest szarpnięciem całą konstrukcją, więc wartość musi
  // być ustawialna z URDF-a. Domyślne 0 zachowuje dotychczasowe zachowanie.
  int profile_velocity_{0};
  int profile_acceleration_{0};
  static constexpr double kProtocolVersion = 2.0;
  static constexpr int32_t kEncoderResolution = 4096;

  std::vector<JointHandle> joints_;

  dynamixel::PortHandler * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};

  rclcpp::Logger logger_{rclcpp::get_logger("DynamixelSystem")};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_
