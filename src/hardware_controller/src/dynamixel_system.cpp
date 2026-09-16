#include "hardware_controller/dynamixel_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

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
    // Profil startowy = ten z URDF-a, żeby kontroler profilu, który jeszcze nic
    // nie wysłał, zostawiał dotychczasowe zachowanie.
    jh.command_profile_velocity = profile_velocity_ * kDegPerSecPerProfileVelUnit;
    jh.command_profile_acceleration = profile_acceleration_ * kDegPerSec2PerProfileAccUnit;
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
    interfaces.emplace_back(joint.name, "profile_velocity", &joint.command_profile_velocity);
    interfaces.emplace_back(joint.name, "profile_acceleration", &joint.command_profile_acceleration);
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
    joint.written_profile_velocity_raw = profile_velocity_;
    joint.written_profile_acceleration_raw = profile_acceleration_;
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

  // Grupy Sync Read / Sync Write - uzasadnienie w nagłówku klasy.
  sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_handler_, packet_handler_,
    DynamixelRegisters::SYNC_READ_START, DynamixelRegisters::SYNC_READ_LENGTH);
  sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
    port_handler_, packet_handler_, DynamixelRegisters::ADDR_GOAL_POSITION, 4);
  for (const auto & joint : joints_)
  {
    if (!sync_read_->addParam(joint.servo_id))
    {
      RCLCPP_ERROR(logger_, "Nie można dodać serwa ID=%d do Sync Read", joint.servo_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  cycle_ = 0;
  sync_read_failures_ = 0;
  stale_cycles_ = 0;

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
    joint.written_goal_raw = INT32_MIN;   // pierwszy write() wyśle cel = pozycja
  }
  cycles_since_goal_write_ = 0;
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
  RCLCPP_INFO(
    logger_, "Magistrala serw: %lu cykli, %lu nieudanych prób Sync Read, %lu cykli z nieaktualnym stanem",
    static_cast<unsigned long>(cycle_), static_cast<unsigned long>(sync_read_failures_),
    static_cast<unsigned long>(stale_cycles_));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // JEDEN pakiet na pozycję, prędkość i prąd wszystkich serw.
  //
  // UWAGA na semantykę SDK (sprawdzone w źródłach dynamixel_sdk 4.0.3): po
  // DOWOLNYM błędzie grupy isAvailable() zwraca false dla WSZYSTKICH serw,
  // także tych, które zdążyły odpowiedzieć - flaga last_result jest wspólna.
  // Zgubiona odpowiedź jednego serwa unieważnia więc cały cykl odczytu.
  // Nie obchodzimy tego: obie osie i tak powinny pochodzić z tej samej chwili,
  // a częściowo zaktualizowany stan (jedna oś świeża, druga stara) byłby
  // gorszy dla rekonstrukcji niż dwie jednakowo nieaktualne.
  //
  // PONOWIENIE W TYM SAMYM CYKLU. Po przejściu na Sync Read zostało 1,2 błędu/s
  // (zmierzone 2026-09-17), ale każdy unieważnia CAŁY cykl, czyli ~2,4% próbek
  // /joint_states niosło starą pozycję ze świeżym stemplem - w skanie ciągłym
  // to 0,5 st błędu kąta na próbkę. Jedna transakcja trwa ~5 ms przy budżecie
  // 20 ms, a ponawiamy tylko po porażce, więc koszt pojawia się wyłącznie tam,
  // gdzie jest potrzebny. Przy niezależnych błędach zostaje ~0,06% cykli.
  int result = COMM_TX_FAIL;
  for (int attempt = 0; attempt < kSyncReadAttempts; ++attempt)
  {
    result = sync_read_->txRxPacket();
    if (result == COMM_SUCCESS)
    {
      break;
    }
    ++sync_read_failures_;
  }
  if (result != COMM_SUCCESS)
  {
    ++stale_cycles_;
    log_comm_error(0, result);
  }

  for (auto & joint : joints_)
  {
    // Stan aktualizujemy TYLKO z poprawnie odebranych danych. Przy błędzie
    // zostaje ostatnia znana wartość - lepsza chwilowo nieaktualna próbka niż
    // zero udające realny pomiar.
    if (!sync_read_->isAvailable(
        joint.servo_id, DynamixelRegisters::SYNC_READ_START, DynamixelRegisters::SYNC_READ_LENGTH))
    {
      continue;
    }

    const int32_t pos_raw = static_cast<int32_t>(
      sync_read_->getData(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_POSITION, 4));
    joint.state_position = raw_to_rad(pos_raw, joint.center_raw);

    // velocity/effort surowe, bez konwersji na jednostki SI — tak samo jak
    // dotychczasowy dynamixel_node.py (JointState.velocity/effort = raw).
    joint.state_velocity = static_cast<double>(static_cast<int32_t>(
      sync_read_->getData(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_VELOCITY, 4)));

    // PRESENT_CURRENT jest w rejestrze liczbą ZE ZNAKIEM (int16) — znak niesie
    // kierunek momentu. Bez tego rzutowania prąd przeciwnego znaku wychodził
    // jako ~65500 zamiast małej wartości ujemnej, co psuło każdy pomiar
    // obciążenia. Jednostka zostaje surowa (1 = 2,69 mA dla XM540).
    joint.state_effort = static_cast<double>(static_cast<int16_t>(static_cast<uint16_t>(
      sync_read_->getData(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_CURRENT, 2))));
  }

  // Temperatura: jedno serwo co kTemperatureEvery cykli, na zmianę.
  if (!joints_.empty() && (cycle_ % kTemperatureEvery) == 0)
  {
    auto & joint = joints_[(cycle_ / kTemperatureEvery) % joints_.size()];
    bool ok = false;
    const uint8_t tmp_raw = read1(joint.servo_id, DynamixelRegisters::ADDR_PRESENT_TEMPERATURE, ok);
    if (ok)
    {
      joint.state_temperature = static_cast<double>(tmp_raw);
    }
  }
  ++cycle_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Sync Write jest rozgłoszeniowy: serwa NIE odsyłają statusu, więc nie ma na
  // co czekać ani czego zgubić po stronie odbioru. Jedyny koszt to brak
  // potwierdzenia - a to i tak widać w następnym odczycie pozycji.
  //
  // UWAGA: znika przez to komunikat "data value exceeds the limit" przy celu
  // spoza okna limitów w EEPROM (tak wyszła kolumna az=180 w 2026-09-02, 5916
  // błędów). Teraz serwo odrzuca taki cel PO CICHU i po prostu stoi. Wykryje to
  // tylko porównanie zadanej pozycji z odczytaną - w sweepie robi to warunek
  // dojazdu ("Brak dojazdu ... Sprawdź limity w EEPROM").
  // Profil PRZED celem: serwo planuje trajektorię w chwili przyjęcia Goal
  // Position, więc cel wysłany przed nowym profilem pojechałby starym.
  for (auto & joint : joints_)
  {
    write_profile_if_changed(joint);
  }

  // CEL TYLKO PRZY ZMIANIE (od 2026-09-17). Każdy zapis Goal Position każe
  // firmware'owi PRZELICZYĆ profil od bieżącej prędkości. Zapis w każdym cyklu
  // (50 Hz) przy małym przyspieszeniu profilu dławił ruch: przyrost prędkości
  // między przeliczeniami ginął w zaokrągleniu do całych jednostek i oś
  // jechała na przypadkowym "progu". Zmierzone: rejestr profile_velocity = 9
  // (12,4 st/s), a oś 4,1 st/s w jednym ruchu i 8,2 st/s w kolejnym, przy
  // identycznych komendach. Cel wysłany raz = trapez zaplanowany raz.
  //
  // Sync Write nie ma potwierdzenia, więc zgubiony pakiet znaczyłby cel, który
  // nigdy nie dotarł. Ponawiamy go, ale TYLKO gdy serwo STOI z dala od celu -
  // ponowienie w trakcie ruchu samo wywołałoby opisane wyżej przeliczenie.
  //
  // Do pakietu trafiają WYŁĄCZNIE osie, które tego wymagają. Ponowny zapis
  // niezmienionego celu osi będącej w ruchu przeliczyłby jej profil tak samo.
  const bool resend_window = cycles_since_goal_write_ >= kGoalResendAfterCycles;
  ++cycles_since_goal_write_;
  sync_write_->clearParam();
  bool any = false;
  for (auto & joint : joints_)
  {
    const int32_t goal = rad_to_raw(joint.command_position, joint.center_raw);
    const bool changed = (goal != joint.written_goal_raw);
    const bool lost = resend_window && joint.state_velocity == 0.0 &&
      std::abs(rad_to_raw(joint.state_position, joint.center_raw) - goal) > kGoalResendToleranceRaw;
    if (!changed && !lost)
    {
      continue;
    }
    uint8_t data[4] = {
      DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)),
      DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))};
    sync_write_->addParam(joint.servo_id, data);
    joint.written_goal_raw = goal;
    any = true;
  }
  if (!any)
  {
    return hardware_interface::return_type::OK;
  }
  const int result = sync_write_->txPacket();
  if (result != COMM_SUCCESS)
  {
    log_comm_error(0, result);
  }
  cycles_since_goal_write_ = 0;
  return hardware_interface::return_type::OK;
}

void DynamixelSystem::write_profile_if_changed(JointHandle & joint)
{
  // TYLKO PRZY ZMIANIE, a nie co cykl: to pojedyncze transakcje z odpowiedzią
  // (~2,5 ms każda), a pętla po przejściu na Sync Read mieści się w budżecie
  // właśnie dlatego, że nie robi nic zbędnego. Profil zmienia się kilka razy na
  // wiersz skanu, nie 50 razy na sekundę.
  const auto to_raw = [](double value, double unit) -> int32_t {
    if (!std::isfinite(value) || value <= 0.0)
    {
      return -1;                         // ignoruj - patrz komentarz w JointHandle
    }
    // Co najmniej 1: surowe 0 znaczy "bez profilu", czyli pełna prędkość.
    return std::max<int32_t>(1, static_cast<int32_t>(std::lround(value / unit)));
  };

  const int32_t vel_raw = to_raw(joint.command_profile_velocity, kDegPerSecPerProfileVelUnit);
  if (vel_raw > 0 && vel_raw != joint.written_profile_velocity_raw)
  {
    // Zapamiętujemy TYLKO udany zapis - nieudany ponowi się w następnym cyklu.
    if (write4(joint.servo_id, DynamixelRegisters::ADDR_PROFILE_VELOCITY, vel_raw))
    {
      joint.written_profile_velocity_raw = vel_raw;
    }
  }

  const int32_t acc_raw = to_raw(joint.command_profile_acceleration, kDegPerSec2PerProfileAccUnit);
  if (acc_raw > 0 && acc_raw != joint.written_profile_acceleration_raw)
  {
    if (write4(joint.servo_id, DynamixelRegisters::ADDR_PROFILE_ACCELERATION, acc_raw))
    {
      joint.written_profile_acceleration_raw = acc_raw;
    }
  }
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

bool DynamixelSystem::write4(uint8_t servo_id, uint16_t addr, int32_t value)
{
  uint8_t error = 0;
  const int result = packet_handler_->write4ByteTxRx(
    port_handler_, servo_id, addr, static_cast<uint32_t>(value), &error);
  if (result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(
      logger_, "Błąd komunikacji z serwem ID=%d: %s", servo_id,
      packet_handler_->getTxRxResult(result));
    return false;
  }
  if (error != 0)
  {
    RCLCPP_WARN(
      logger_, "Błąd pakietu serwa ID=%d: %s", servo_id, packet_handler_->getRxPacketError(error));
    return false;
  }
  return true;
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
  // servo_id 0 = transakcja GRUPOWA (Sync Read/Write) - adres 0 nie jest
  // używany przez żadne serwo w projekcie, więc nie ma dwuznaczności.
  if (servo_id == 0)
  {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "Błąd transakcji grupowej: %s (nieudanych prób Sync Read: %lu, cykli z NIEAKTUALNYM "
      "stanem mimo ponowienia: %lu)",
      packet_handler_->getTxRxResult(result), static_cast<unsigned long>(sync_read_failures_),
      static_cast<unsigned long>(stale_cycles_));
    return;
  }
  RCLCPP_ERROR_THROTTLE(
    logger_, *clock_, 2000,
    "Błąd komunikacji z serwem ID=%d: %s (łącznie błędów: %lu)", servo_id,
    packet_handler_->getTxRxResult(result), static_cast<unsigned long>(comm_error_count_));
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::DynamixelSystem, hardware_interface::SystemInterface)
