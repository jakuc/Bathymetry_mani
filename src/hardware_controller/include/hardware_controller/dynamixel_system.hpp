#ifndef HARDWARE_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_
#define HARDWARE_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_

#include <climits>
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
  // Nastawy pętli położenia. UWAGA: to rejestry RAM, więc każde odłączenie
  // zasilania przywraca fabryczne 800/0/0 - dlatego zapisujemy je przy każdej
  // konfiguracji, a nie zakładamy, że przetrwały z poprzedniej sesji.
  static constexpr uint16_t ADDR_POSITION_D_GAIN = 80;
  static constexpr uint16_t ADDR_POSITION_I_GAIN = 82;
  static constexpr uint16_t ADDR_POSITION_P_GAIN = 84;
  static constexpr uint8_t MODE_POSITION = 3;

  // Blok SYNC READ: Present Current (126, 2 B) + Present Velocity (128, 4 B)
  // + Present Position (132, 4 B) leżą w tablicy rejestrów obok siebie, więc
  // jeden pakiet zbiera wszystkie trzy dla wszystkich serw naraz.
  static constexpr uint16_t SYNC_READ_START = 126;
  static constexpr uint16_t SYNC_READ_LENGTH = 10;
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
  // PROFIL RUCHU ZMIENIANY W LOCIE (od 2026-09-17), w jednostkach fizycznych.
  //
  // Po co: skan ciągły strumieniował pozycję co 20 ms, a serwo z włączonym
  // profilem planowało przy KAŻDYM kroku osobny mini-trapez (rozpęd, dojście,
  // hamowanie) - user zobaczył to jako "klatkowanie" głowicy. Zamiast tego
  // ustawiamy prędkość i przyspieszenie profilu, wysyłamy SAM PUNKT KOŃCOWY,
  // a trajektorię liczy firmware serwa z częstotliwością 1 kHz.
  //
  // Jednostki fizyczne, a nie surowe, bo surowe znaczą co innego zależnie od
  // drive_mode (patrz xm540.ros2_control.xacro) i łatwo je pomylić.
  // Wartość <= 0 albo NaN jest IGNOROWANA: surowe 0 u Dynamixela znaczy
  // "bez profilu", czyli skok z pełną prędkością - nigdy nie wolno go wysłać
  // przypadkiem.
  double command_profile_velocity{0.0};       // st/s
  double command_profile_acceleration{0.0};   // st/s^2
  int32_t written_profile_velocity_raw{-1};   // ostatnio ZAPISANE do serwa
  int32_t written_profile_acceleration_raw{-1};
  // Ostatnio WYSŁANY cel - Goal Position zapisujemy tylko przy zmianie, patrz write().
  int32_t written_goal_raw{INT32_MIN};
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
  void write2(uint8_t servo_id, uint16_t addr, uint16_t value);
  // true = serwo potwierdziło zapis bez błędu (potrzebne tam, gdzie nieudany
  // zapis trzeba ponowić, np. profil ruchu).
  bool write4(uint8_t servo_id, uint16_t addr, int32_t value);
  // Odczyty raportują powodzenie przez `ok`. Bez tego nieudana transakcja była
  // nie do odróżnienia od poprawnie odczytanego zera i wchodziła do stanu jointa
  // jako realna wartość - dla pozycji oznaczało to skok o cały offset (na naszym
  // sprzęcie -234 stopnie) w ~1,6% próbek, co psuło każdy pomiar.
  uint8_t read1(uint8_t servo_id, uint16_t addr, bool & ok);
  uint16_t read2(uint8_t servo_id, uint16_t addr, bool & ok);
  int32_t read4(uint8_t servo_id, uint16_t addr, bool & ok);
  void log_comm_error(uint8_t servo_id, int result);

  std::string device_port_;
  int baud_rate_{1000000};
  // Profil ruchu serwa. UWAGA: 0 u Dynamixela nie znaczy "zero prędkości", tylko
  // "bez profilu" — serwo skacze do zadanej pozycji z maksymalną prędkością.
  // Dla samego manipulatora to było nieszkodliwe, ale z zamontowaną głowicą
  // każdy skok komendy jest szarpnięciem całą konstrukcją, więc wartość musi
  // być ustawialna z URDF-a. Domyślne 0 zachowuje dotychczasowe zachowanie.
  int profile_velocity_{0};
  int profile_acceleration_{0};
  // Wartości fabryczne XM540; realne nastawy podaje URDF (patrz xm540.ros2_control.xacro).
  int position_p_gain_{800};
  int position_i_gain_{0};
  int position_d_gain_{0};
  static constexpr double kProtocolVersion = 2.0;
  static constexpr int32_t kEncoderResolution = 4096;
  // Jednostki rejestrów profilu przy drive_mode = 0 (profil PRĘDKOŚCIOWY):
  //   Profile Velocity     1 = 0,229 obr/min      = 1,374 st/s
  //   Profile Acceleration 1 = 214,577 obr/min^2  = 21,4577 st/s^2
  // Przy drive_mode = 4 (profil czasowy) te same rejestry znaczą milisekundy
  // i ta konwersja byłaby BŁĘDNA - oba serwa mają drive_mode = 0 od 2026-08-20.
  static constexpr double kDegPerSecPerProfileVelUnit = 0.229 * 360.0 / 60.0;
  static constexpr double kDegPerSec2PerProfileAccUnit = 214.577 * 360.0 / 3600.0;
  // Zapis profilu tylko przy zmianie - uzasadnienie przy write().
  void write_profile_if_changed(JointHandle & joint);

  std::vector<JointHandle> joints_;

  dynamixel::PortHandler * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};

  // SYNC READ / SYNC WRITE zamiast pojedynczych transakcji (od 2026-09-17).
  //
  // Zmierzone na płytce: pętla robiła 10 transakcji na cykl (4 odczyty na
  // serwo + zapis celu na serwo), każda ~2,5 ms, czyli ~25 ms przy budżecie
  // 20 ms. Magistrala pracowała bez przerwy, /joint_states szło 39-42 Hz
  // z przerwami do 85 ms, a błędy "Incorrect status packet" leciały w stałym
  // tempie 4,3/s - IDENTYCZNIE z dalmierzem i bez niego (A/B tego samego dnia),
  // czyli to nie USB, tylko przeładowana pętla.
  //
  // Przy skanie ciągłym ma to trzy skutki, nie jeden:
  //   1. Nieudany odczyt zostawia STARĄ pozycję, a joint_state_broadcaster
  //      publikuje ją ze ŚWIEŻYM stemplem - przy 24 st/s to 0,5 st błędu kąta.
  //   2. Osie były czytane w odstępie kilku transakcji, więc azymut i elewacja
  //      w jednym /joint_states pochodziły z różnych chwil.
  //   3. Rzadszy /joint_states to rzadsza siatka TF do interpolacji.
  // Sync Read zbiera pozycję, prędkość i prąd wszystkich serw JEDNYM pakietem
  // (serwa odpowiadają kolejno, ale na to samo zapytanie), a Sync Write jest
  // rozgłoszeniowy i nie czeka na odpowiedź. Zostają 2 transakcje na cykl
  // (plus odczyt temperatury co kTemperatureEvery cykli).
  std::unique_ptr<dynamixel::GroupSyncRead> sync_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_;
  // Temperatura zmienia się w skali minut - czytamy ją co tyle cykli, po jednym
  // serwie naraz, żeby nie dokładać transakcji do każdego obiegu pętli.
  static constexpr uint64_t kTemperatureEvery = 50;
  uint64_t cycle_{0};
  // Ile razy próbujemy Sync Read w jednym cyklu - uzasadnienie przy read().
  static constexpr int kSyncReadAttempts = 2;
  uint64_t sync_read_failures_{0};   // każda nieudana PRÓBA
  uint64_t stale_cycles_{0};         // cykle, w których stan został NIEAKTUALNY
  uint64_t cycles_since_goal_write_{0};
  // Po tylu cyklach bez zapisu celu wolno go ponowić - ale tylko serwu, które
  // STOI z dala od celu (zgubiony pakiet Sync Write). Patrz write().
  static constexpr uint64_t kGoalResendAfterCycles = 25;
  static constexpr int32_t kGoalResendToleranceRaw = 3;

  rclcpp::Logger logger_{rclcpp::get_logger("DynamixelSystem")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
  uint64_t comm_error_count_{0};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__DYNAMIXEL_SYSTEM_HPP_
