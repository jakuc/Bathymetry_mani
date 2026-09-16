#ifndef HARDWARE_CONTROLLER__M703A_LASER_SENSOR_HPP_
#define HARDWARE_CONTROLLER__M703A_LASER_SENSOR_HPP_

#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_controller
{

// Dalmierz JRT M703A na głowicy, przez mostek na Arduino Nano
// (firmware/jrt_bridge), W TRYBIE CIĄGŁYM ZE STEMPLAMI CZASU Z NANO.
//
// DLACZEGO TO OSOBNA WTYCZKA, A NIE PARAMETR W JrtLaserSensor
//
//   1. Inny protokół. B87A/LDB1 to ramki binarne (AA ... suma) w schemacie
//      ŻĄDANIE-ODPOWIEDŹ. M703A gada ASCII, komendami jednobajtowymi bez
//      terminatora, a w trybie ciągłym sam sypie liniami ": 1.690m,0071".
//   2. Inna konstrukcja czasu. Tam każdy pomiar miał swój `shot_start`, bo host
//      go zamawiał. Tu host nie zamawia NICZEGO - moduł strzela własnym rytmem
//      (123 ms w trybie F), więc jedyną kotwicą jest stempel micros() z Nano.
//   3. Inny rytm. 8,15 Hz zamiast 1,9 Hz. To jest cały powód wymiany czujnika:
//      półsfera co 3 stopnie schodzi z ~50 min do ~7,5 min.
//
// SKĄD SIĘ BIERZE CZAS POMIARU - najważniejsza rzecz w tej klasie
//
// Punkt chmury powstaje z pary (odległość, kąt osi). Kąt bierze się z TF po
// stemplu czasu, więc BŁĄD CZASU JEST BŁĘDEM KĄTA: przy 24 st/s każde 10 ms to
// 0,24 stopnia. Czas przyjścia linii na hoście jest do tego za słaby - między
// modułem a `read()` siedzi CH340, stos USB, WiFi i pętla 50 Hz, a zmierzony
// jitter to mediana 0,9 ms i ogon do 3,3 ms. Dlatego linia jest stemplowana na
// Nano (`micros()` przy pierwszym bajcie) i to ten stempel wyznacza czas.
//
// Zegar Nano jest jednak rezonatorem ceramicznym i DRYFUJE ~3100 ppm, czyli
// 3 ms na sekundę - po minucie to już 0,2 s, więc stałego przelicznika użyć się
// nie da. Wtyczka estymuje relację zegarów ONLINE, z DOLNEJ OBWIEDNI różnicy
// (czas hosta - stempel Nano): opóźnienie transportu jest zawsze dodatnie, więc
// prawdę o zegarach niosą próbki najmniej opóźnione, a nie średnia (średnia
// byłaby przesunięta o średni jitter i zmieniałaby się z obciążeniem płytki).
//
// Obwiednia liczona jest po koszykach: okno dzielone na kilka przedziałów
// czasu, z każdego brane minimum, przez te minima prosta metodą najmniejszych
// kwadratów. Nachylenie to dryf, wyraz wolny to przesunięcie zegarów.
//
// KOTWICA (anchor_offset_ms): gdzie W OKNIE POMIARU leży chwila, do której
// odnosi się stempel. Dla poprzedniego dalmierza wyszło 0,60 długości okna
// (test rewersyjny, 2026-09-05). Dla M703A to jeszcze NIE JEST ZMIERZONE, więc
// domyślnie 0: czas pomiaru = stempel pierwszego bajtu linii. Poprawkę wpisuje
// się po teście rewersyjnym, bez ruszania kodu.
//
// CZEGO TU NIE MA I DLACZEGO: bramki "zbieraj tylko w bezruchu". Tryb ciągły
// istnieje po to, żeby głowica się NIE zatrzymywała; sortowanie pomiarów na
// dobre i złe robi się czasem, nie postojem.
class M703aLaserSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- transport ----------------------------------------------------------
  bool open_port();
  void write_raw(const std::vector<uint8_t> & bytes);
  void bridge_cmd(uint8_t cmd, uint8_t arg);
  void bridge_cmd(uint8_t cmd);
  size_t drain(std::string & sink, int wait_ms);
  // Zbiera z portu wszystko, co przyszło, i wydziela kompletne linie.
  void pump_lines(std::vector<std::string> & out);

  // --- protokół -----------------------------------------------------------
  // "$<t_first>,<t_last>,<treść>" -> true; treść bez stempli w out_body.
  static bool parse_stamped(const std::string & line, uint32_t & t_first, uint32_t & t_last,
                            std::string & out_body);
  // ": 1.690m,0071" albo "F: 1.689m,0085" -> metry i SQ.
  static bool parse_measurement(const std::string & body, double & out_m, int & out_sq);
  // Rozwija uint32 mikrosekund Nano (przewija się co ~71,6 min) w ciągłą oś.
  double unwrap_nano(uint32_t raw_us);

  // --- zegary -------------------------------------------------------------
  void clock_add(double nano_s, double host_s);
  bool clock_ready() const;
  double clock_to_host(double nano_s) const;

  // Ustawienie modułu w tryb ciągły: stemple, nCTRL w dół, litera trybu.
  void start_stream();

  double get_param(const std::string & name, double fallback) const;
  std::string get_param_str(const std::string & name, const std::string & fallback) const;

  std::string sensor_name_;
  std::string device_port_;
  int usb_baud_{115200};      // host <-> mostek
  int module_baud_{19200};    // mostek <-> moduł (nominał M703A)
  int fd_{-1};

  // "F" (fast, 8,15 Hz), "D" (auto, 2,5 Hz), "M" (slow). Do skanowania F.
  std::string measure_mode_{"F"};
  // Otwarcie portu szarpie DTR i RESETUJE Nano - tyle trwa bootloader.
  int settle_ms_{2500};
  double min_range_{0.03};
  double max_range_{100.0};
  // Patrz komentarz o kotwicy w nagłówku klasy.
  double anchor_offset_s_{0.0};
  // Po tylu ms ciszy uznajemy strumień za zerwany i wznawiamy tryb ciągły.
  int restart_after_ms_{1500};

  // Okno estymacji zegara i liczba koszyków obwiedni.
  double clock_window_s_{120.0};
  int clock_buckets_{8};

  std::string rx_;                       // niedokończona linia
  std::deque<std::pair<double, double>> clock_;   // (czas Nano [s], host - Nano [s])
  double clock_drift_{0.0};              // nachylenie: sekundy hosta na sekundę Nano - 1
  double clock_offset_{0.0};
  bool clock_valid_{false};

  bool have_epoch_{false};
  uint32_t last_raw_us_{0};
  double nano_epoch_{0.0};               // narosłe przewinięcia [s]
  double nano_first_{0.0};               // pierwszy stempel, zero osi Nano

  // Pierwsza linia po komendzie trybu jest ROZCIĄGNIĘTA (~300 ms): moduł odsyła
  // literę komendy od razu, a resztę dopiero po pierwszym pomiarze. Jej stempel
  // to chwila przyjęcia komendy, nie pomiaru - odrzucamy ją.
  bool skip_first_{true};

  rclcpp::Time last_line_stamp_;
  bool have_line_{false};

  double state_range_{0.0};
  double state_signal_quality_{0.0};
  double state_status_code_{0.0};
  double state_sample_time_{0.0};
  // Zachowane dla zgodności z LaserBroadcaster i kolektorem: tu = czas pomiaru
  // PRZED poprawką kotwicy, czyli surowy stempel Nano przeliczony na zegar ROS.
  double state_shot_start_{0.0};

  uint64_t lines_{0};
  uint64_t bad_lines_{0};
  uint64_t restarts_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("M703aLaserSensor")};
  rclcpp::Clock clock_throttle_{RCL_STEADY_TIME};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__M703A_LASER_SENSOR_HPP_
