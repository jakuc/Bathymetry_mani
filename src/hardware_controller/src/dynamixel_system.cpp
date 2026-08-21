#include "hardware_controller/dynamixel_system.hpp"

#include <cmath>
#include <cstdint>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

hardware_interface::CallbackReturn DynamixelSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters.at("device_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));

  // Opcjonalne — brak wpisu w URDF-ie zostawia 0, czyli ruch bez profilu.
  const auto opt_param = [this](const std::string & key, int fallback) {
    const auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? std::stoi(it->second) : fallback;
  };
  profile_velocity_ = opt_param("profile_velocity", profile_velocity_);
  profile_acceleration_ = opt_param("profile_acceleration", profile_acceleration_);
  position_p_gain_ = opt_param("position_p_gain", position_p_gain_);
  position_i_gain_ = opt_param("position_i_gain", position_i_gain_);
  position_d_gain_ = opt_param("position_d_gain", position_d_gain_);

  joints_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints)
  {
    JointHandle jh;
    jh.name = joint.name;
    jh.servo_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("servo_id")));
    const auto center_it = joint.parameters.find("center_raw");
    if (center_it != joint.parameters.end())
    {
      jh.center_raw = std::stoi(center_it->second);
    }
    joints_.push_back(jh);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : joints_)
  {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.state_effort);
    interfaces.emplace_back(joint.name, "temperature", &joint.state_temperature);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : joints_)
  {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.command_position);
  }
  return interfaces;
}

hardware_interface::CallbackReturn DynamixelSystem::on_configure(const rclcpp_lifecycle::State &)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);

  if (!port_handler_->openPort())
  {
    RCLCPP_ERROR(logger_, "Nie można otworzyć portu %s", device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!port_handler_->setBaudRate(baud_rate_))
  {
    RCLCPP_ERROR(logger_, "Nie można ustawić baud rate %d na %s", baud_rate_, device_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_)
  {
    write1(joint.servo_id, DynamixelRegisters::ADDR_TORQUE_ENABLE, 0);
    write1(joint.servo_id, DynamixelRegisters::ADDR_OPERATING_MODE, DynamixelRegisters::MODE_POSITION);
    write4(joint.servo_id, DynamixelRegisters::ADDR_PROFILE_VELOCITY, profile_velocity_);
    write4(joint.servo_id, DynamixelRegisters::ADDR_PROFILE_ACCELERATION, profile_acceleration_);
    // Nastawy pętli położenia zapisujemy przy KAŻDEJ konfiguracji, bo to rejestry
    // RAM: odłączenie zasilania przywraca fabryczne 800/0/0. Fabryczne nastawy
    // zostawiają uchyb ustalony 0,31 st na elewacji i 0,75 st na azymucie (pomiar
    // 2026-08-17) - przy czystym P oś staje tam, gdzie moment członu P równoważy
    // tarcie, więc uchybu nie da się usunąć inaczej niż większym P i członem I.
    write2(
      joint.servo_id, DynamixelRegisters::ADDR_POSITION_P_GAIN,
      static_cast<uint16_t>(position_p_gain_));
    write2(
      joint.servo_id, DynamixelRegisters::ADDR_POSITION_I_GAIN,
      static_cast<uint16_t>(position_i_gain_));
    write2(
      joint.servo_id, DynamixelRegisters::ADDR_POSITION_D_GAIN,
      static_cast<uint16_t>(position_d_gain_));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & joint : joints_)
  {
    write1(joint.servo_id, DynamixelRegisters::ADDR_TORQUE_ENABLE, 1);
    // Komenda startowa = aktualna pozycja serwa, żeby aktywacja nie szarpnęła
    // manipulatorem. Nieudany odczyt jest tu GROŹNY: przyjęcie zera oznaczałoby
    // komendę "jedź do zera enkodera" z pełną konstrukcją, więc wolimy nie wstać.
    // Ponawiamy, bo pojedynczy zgubiony pakiet to na tej magistrali norma i nie
    // ma powodu, żeby przez niego nie wstał cały stack.
    bool ok = false;
    int32_t raw = 0;
    for (int attempt = 0; attempt < 5 && !ok; ++attempt)
    {
      raw = read4(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_POSITION, ok);
    }
    if (!ok)
    {
      RCLCPP_ERROR(
        logger_, "Nie mogę odczytać pozycji startowej serwa ID=%d — przerywam aktywację",
        joint.servo_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint.state_position = raw_to_rad(raw, joint.center_raw);
    joint.command_position = joint.state_position;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & joint : joints_)
  {
    write1(joint.servo_id, DynamixelRegisters::ADDR_TORQUE_ENABLE, 0);
  }
  if (port_handler_)
  {
    port_handler_->closePort();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : joints_)
  {
    // Każdy odczyt aktualizuje stan TYLKO wtedy, gdy transakcja się powiodła.
    // Przy błędzie zostaje ostatnia znana wartość - lepsza jest chwilowo
    // nieaktualna próbka niż zero udające realny pomiar.
    bool ok = false;

    const int32_t pos_raw = read4(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_POSITION, ok);
    if (ok)
    {
      joint.state_position = raw_to_rad(pos_raw, joint.center_raw);
    }

    // velocity/effort surowe, bez konwersji na jednostki SI — tak samo jak
    // dotychczasowy dynamixel_node.py (JointState.velocity/effort = raw).
    const int32_t vel_raw = read4(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_VELOCITY, ok);
    if (ok)
    {
      joint.state_velocity = static_cast<double>(vel_raw);
    }

    // PRESENT_CURRENT jest w rejestrze liczbą ZE ZNAKIEM (int16) — znak niesie
    // kierunek momentu. Bez tego rzutowania prąd przeciwnego znaku wychodził
    // jako ~65500 zamiast małej wartości ujemnej, co psuło każdy pomiar
    // obciążenia. Jednostka zostaje surowa (1 = 2,69 mA dla XM540).
    const uint16_t cur_raw = read2(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_CURRENT, ok);
    if (ok)
    {
      joint.state_effort = static_cast<double>(static_cast<int16_t>(cur_raw));
    }

    const uint8_t tmp_raw = read1(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_TEMPERATURE, ok);
    if (ok)
    {
      joint.state_temperature = static_cast<double>(tmp_raw);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : joints_)
  {
    write4(
      joint.servo_id, DynamixelRegisters::ADDR_GOAL_POSITION,
      rad_to_raw(joint.command_position, joint.center_raw));
  }
  return hardware_interface::return_type::OK;
}

double DynamixelSystem::raw_to_rad(int32_t raw, int32_t center_raw) const
{
  return static_cast<double>(raw - center_raw) * 2.0 * M_PI / static_cast<double>(kEncoderResolution);
}

int32_t DynamixelSystem::rad_to_raw(double rad, int32_t center_raw) const
{
  return center_raw + static_cast<int32_t>(std::lround(rad * kEncoderResolution / (2.0 * M_PI)));
}

void DynamixelSystem::write1(uint8_t servo_id, uint16_t addr, uint8_t value)
{
  uint8_t error = 0;
  const int result = packet_handler_->write1ByteTxRx(port_handler_, servo_id, addr, value, &error);
  if (result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(
      logger_, "Błąd komunikacji z serwem ID=%d: %s", servo_id,
      packet_handler_->getTxRxResult(result));
  }
  else if (error != 0)
  {
    RCLCPP_WARN(
      logger_, "Błąd pakietu serwa ID=%d: %s", servo_id, packet_handler_->getRxPacketError(error));
  }
}

void DynamixelSystem::write2(uint8_t servo_id, uint16_t addr, uint16_t value)
{
  uint8_t error = 0;
  const int result = packet_handler_->write2ByteTxRx(port_handler_, servo_id, addr, value, &error);
  if (result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(
      logger_, "Błąd komunikacji z serwem ID=%d: %s", servo_id,
      packet_handler_->getTxRxResult(result));
  }
  else if (error != 0)
  {
    RCLCPP_WARN(
      logger_, "Błąd pakietu serwa ID=%d: %s", servo_id, packet_handler_->getRxPacketError(error));
  }
}

void DynamixelSystem::write4(uint8_t servo_id, uint16_t addr, int32_t value)
{
  uint8_t error = 0;
  const int result = packet_handler_->write4ByteTxRx(
    port_handler_, servo_id, addr, static_cast<uint32_t>(value), &error);
  if (result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(
      logger_, "Błąd komunikacji z serwem ID=%d: %s", servo_id,
      packet_handler_->getTxRxResult(result));
  }
  else if (error != 0)
  {
    RCLCPP_WARN(
      logger_, "Błąd pakietu serwa ID=%d: %s", servo_id, packet_handler_->getRxPacketError(error));
  }
}

uint8_t DynamixelSystem::read1(uint8_t servo_id, uint16_t addr, bool & ok)
{
  uint8_t value = 0;
  uint8_t error = 0;
  const int result = packet_handler_->read1ByteTxRx(port_handler_, servo_id, addr, &value, &error);
  ok = (result == COMM_SUCCESS);
  if (!ok)
  {
    log_comm_error(servo_id, result);
  }
  return value;
}

uint16_t DynamixelSystem::read2(uint8_t servo_id, uint16_t addr, bool & ok)
{
  uint16_t value = 0;
  uint8_t error = 0;
  const int result = packet_handler_->read2ByteTxRx(port_handler_, servo_id, addr, &value, &error);
  ok = (result == COMM_SUCCESS);
  if (!ok)
  {
    log_comm_error(servo_id, result);
  }
  return value;
}

int32_t DynamixelSystem::read4(uint8_t servo_id, uint16_t addr, bool & ok)
{
  uint32_t value = 0;
  uint8_t error = 0;
  const int result = packet_handler_->read4ByteTxRx(port_handler_, servo_id, addr, &value, &error);
  ok = (result == COMM_SUCCESS);
  if (!ok)
  {
    log_comm_error(servo_id, result);
  }
  return static_cast<int32_t>(value);
}

void DynamixelSystem::log_comm_error(uint8_t servo_id, int result)
{
  // Dławione: pojedynczy zgubiony pakiet jest na tej magistrali normą (rzędu
  // procenta), a nieprzytłumiony RCLCPP_ERROR przy 50 Hz i czterech odczytach
  // na serwo zasypuje log setkami linii i przykrywa komunikaty, które naprawdę
  // coś znaczą. Licznik w treści pokazuje skalę zjawiska mimo dławienia.
  ++comm_error_count_;
  RCLCPP_ERROR_THROTTLE(
    logger_, *clock_, 2000,
    "Błąd komunikacji z serwem ID=%d: %s (łącznie błędów: %lu)", servo_id,
    packet_handler_->getTxRxResult(result), static_cast<unsigned long>(comm_error_count_));
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::DynamixelSystem, hardware_interface::SystemInterface)
