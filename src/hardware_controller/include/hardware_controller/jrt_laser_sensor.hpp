#ifndef HARDWARE_CONTROLLER__JRT_LASER_SENSOR_HPP_
#define HARDWARE_CONTROLLER__JRT_LASER_SENSOR_HPP_

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

// Dalmierz laserowy JRT (naklejka "LDB1 A191", rodzina protokołu B87A/M8xx)
// wpięty przez Arduino Nano pracujące jako PRZEZROCZYSTY MOSTEK
// (firmware/jrt_bridge). Zastępuje Sharpa GP2Y0A710K0F na głowicy.
//
// CZYM TO SIĘ RÓŻNI OD SHARPA - i dlaczego to osobna wtyczka, a nie parametr
// w LaserSensor:
//
//   1. Sharp był czujnikiem ANALOGOWYM i STRUMIENIOWYM: Nano samo w kółko
//      próbkowało ADC i sypało ramkami, host tylko czytał. JRT jest cyfrowy i
//      pracuje w trybie ŻĄDANIE-ODPOWIEDŹ - dopóki nie wyślesz komendy, milczy.
//   2. Sharp wymagał kalibracji (tabela PCHIP, bo krzywa producenta się nie
//      zgadzała). JRT zwraca MILIMETRY. Nie ma czego kalibrować i nie wolno
//      niczego dopasowywać - własna korekta tylko zepsułaby fabryczną.
//   3. Sharp miał ~18,6 Hz. Tu pojedynczy strzał trwa ~0,52 s w trybie fast i
//      ~3,3 s w slow (zmierzone 2026-08-30).
//
// PUNKT 3 WYZNACZA CAŁĄ KONSTRUKCJĘ TEJ KLASY. Pętla ros2_control kręci się
// 50 Hz, czyli read() ma 20 ms i NIE WOLNO w nim czekać na odpowiedź czujnika -
// zablokowanie na pół sekundy zatrzymałoby razem z nim sterowanie serwami.
// Dlatego read() jest MASZYNĄ STANÓW bez ani jednego sleepa: w jednym cyklu
// wysyła żądanie, przez kolejne kilkadziesiąt dobiera bajty odpowiedzi, składa
// ramkę i wraca do stanu wyjściowego. Z zewnątrz wygląda to jak czujnik
// strumieniowy o częstotliwości ~2 Hz.
//
// PROTOKÓŁ (zweryfikowany na żywo 2026-08-29/30):
//   ramka       AA <adres> <rejestr 2B> <licznik słów 2B> <dane...> <suma>
//   suma        = (suma bajtów PO 0xAA) & 0xFF
//   pomiar fast AA 00 00 20 00 01 00 02 23
//   odpowiedź   AA 00 00 22 00 03 <dystans 4B big-endian, mm> <SQ 2B> <suma>
//   błąd        EE 00 00 00 00 01 <kod 2B> <suma>
//
// JAKOŚĆ SYGNAŁU (SQ) JEST ODWROTNA NIŻ INTUICJA: mniejsza liczba = mocniejszy
// sygnał. Na twardej ścianie z 1,7 m wychodziło 15-40.
//
// TRZY RZECZY, KTÓRE POTRAFIĄ TU DAĆ CISZĘ NIE DO ODRÓŻNIENIA OD AWARII -
// wszystkie załatwione w on_configure, żeby nie wracały:
//   - PWREN (D4 mostka) domyślnie niski = moduł WYŁĄCZONY;
//   - baud modułu to 38400, a nie 19200 z dokumentacji rodziny;
//   - ramki dłuższe niż 5 bajtów rozjeżdżają się, jeśli mostek przepisuje je
//     bajt po bajcie (SoftwareSerial gasi przerwania na czas nadawania).
//     Dlatego KAŻDA ramka idzie przez bufor mostka: ESC ESC 'W' <len> <dane>.
//
// on_configure NIE KOŃCZY SIĘ SUKCESEM, DOPÓKI MODUŁ SIĘ NIE ODEZWIE (czytamy
// numer seryjny). To celowe: cichy start i NaN-y w nieskończoność są znacznie
// gorsze od jawnego błędu przy starcie stacku.
class JrtLaserSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- warstwa transportu -------------------------------------------------
  bool open_port();
  void write_raw(const std::vector<uint8_t> & bytes);
  // Ramka JRT przez bufor mostka - patrz komentarz o SoftwareSerial wyżej.
  void write_frame(const std::vector<uint8_t> & frame);
  void bridge_cmd(uint8_t cmd);
  void bridge_cmd(uint8_t cmd, uint8_t arg);
  // Kolejny krok odtwarzania łącza; true = zakończone.
  bool step_recovery(const rclcpp::Time & time);
  uint8_t bridge_baud_index() const;
  size_t drain(std::vector<uint8_t> & sink, int wait_ms);

  // --- warstwa protokołu --------------------------------------------------
  static std::vector<uint8_t> make_frame(const std::vector<uint8_t> & payload);
  // Szuka w buforze kompletnej, poprawnej sumą ramki. Zwraca true i wypełnia
  // out_*, zjadając ramkę z bufora. Bajty przed nagłówkiem odrzuca.
  bool extract_frame(uint16_t & out_reg, std::vector<uint8_t> & out_payload, bool & out_is_error);
  // Blokujące odpytanie - WOLNO go używać TYLKO w on_configure, nigdy w read().
  bool query_blocking(const std::vector<uint8_t> & frame, uint16_t expect_reg,
                      std::vector<uint8_t> & payload, int timeout_ms);

  double get_param(const std::string & name, double fallback) const;
  std::string get_param_str(const std::string & name, const std::string & fallback) const;

  std::string sensor_name_;
  std::string device_port_;
  int usb_baud_{115200};       // mostek <-> host
  int module_baud_{38400};     // mostek <-> moduł
  int fd_{-1};

  std::string measure_mode_{"fast"};
  std::vector<uint8_t> measure_frame_;

  // Odstęp między kolejnymi żądaniami. 0 = strzelaj tak szybko, jak moduł
  // nadąża; sensowne, bo to on jest wąskim gardłem, nie pętla.
  int period_ms_{0};
  // Po tylu ms bez odpowiedzi uznajemy strzał za nieudany i strzelamy od nowa.
  // Bez tego jedna zgubiona ramka zawiesiłaby czujnik na zawsze.
  int timeout_ms_{1500};
  // Otwarcie portu szarpie DTR i RESETUJE Nano - tyle trwa bootloader mostka.
  int settle_ms_{2200};

  double min_range_{0.03};
  double max_range_{100.0};

  // ODTWARZANIE ŁĄCZA. Zmierzone 2026-08-30 na płytce: po ~200 udanych strzałach
  // moduł zamilkł i od tego momentu przepadał KAŻDY - 121 nieudanych z 313, przy
  // czym port USB żył, a proces trzymał poprawny deskryptor (sprawdzone w
  // /proc/<pid>/fd), więc to NIE była awaria USB z [[project_usb_dropout_silent_death]].
  //
  // Najpoważniejszy podejrzany: RESET NANO. Mostek wstaje wtedy z domyślnym
  // BAUDS[0] = 19200, a moduł mówi 38400 - i cisza jest już trwała, bo host
  // wysyła indeks baudu TYLKO raz, w on_configure. Dokładnie ten sam objaw daje
  // moduł, który sam wypadł z trybu pracy.
  //
  // Obie przyczyny leczy to samo: powtórzenie sekwencji rozruchowej. Dlatego po
  // kilku nieudanych strzałach wtyczka NIE poddaje się do końca życia procesu,
  // tylko przechodzi w Recovering i odtwarza łącze - rozłożone na kilka cykli
  // read(), bo sekwencja wymaga przerw (~1,2 s), a blokowanie pętli 50 Hz
  // zatrzymałoby razem z nią serwa.
  //
  // Przy wejściu w odtwarzanie pytamy mostek o jego status (1B 1B 3F). Odpowiedź
  // zawiera baud, na którym stoi - i to ROZSTRZYGA, czy Nano się zresetowało.
  // Bez tego zostaje zgadywanie, bo samo otwarcie portu do sprawdzenia i tak
  // resetuje Nano przez DTR, niszcząc dowód.
  enum class Phase { Idle, Waiting, Recovering };
  Phase phase_{Phase::Idle};
  rclcpp::Time request_stamp_;
  rclcpp::Time next_request_stamp_;
  bool have_clock_{false};

  std::vector<uint8_t> rx_;

  double state_range_{0.0};
  double state_signal_quality_{0.0};
  double state_status_code_{0.0};
  double state_sample_time_{0.0};
  // Moment WYSLANIA zadania, ktorego dotyczy biezacy odczyt - nie moment
  // powrotu odpowiedzi. To jest jedyna informacja, ktora pozwala odbiorcy
  // stwierdzic, czy pomiar powstal przy nieruchomej glowicy: czas pomiaru
  // modulu waha sie wg instrukcji 0,1-4 s, wiec z samego czasu POWROTU nie da
  // sie tego wywnioskowac. Bez tego sweep musialby zgadywac timeoutem.
  double state_shot_start_{0.0};

  uint64_t shots_{0};
  uint64_t misses_{0};
  uint64_t recoveries_{0};
  int consecutive_misses_{0};
  int recover_after_misses_{3};
  int recover_stage_{0};
  rclcpp::Time recover_deadline_;

  rclcpp::Logger logger_{rclcpp::get_logger("JrtLaserSensor")};
  // Zegar do RCLCPP_*_THROTTLE. Steady, bo służy tylko do dławienia
  // komunikatów i nie ma nic wspólnego z czasem pomiaru.
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__JRT_LASER_SENSOR_HPP_
