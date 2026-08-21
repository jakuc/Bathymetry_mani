#ifndef HARDWARE_CONTROLLER__LASER_SENSOR_HPP_
#define HARDWARE_CONTROLLER__LASER_SENSOR_HPP_

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

// Dalmierz Sharp GP2Y0A710K0F na Arduino Nano (klon z CH340), firmware
// firmware/laser_nano. Nano nadaje po 115200 8N1 ramki tekstowe
//   LSR,<mediana>,<min>,<max>,<n>
// gdzie wszystkie liczby to SUROWE zliczenia ADC (10 bitów, referencja = Vcc),
// a mediana jest liczona z okna <n> próbek. Przeliczenie na metry siedzi tutaj,
// nie w firmware - patrz komentarz w laser_nano.ino i scripts/laser_test.py.
//
// PRZELICZENIE ADC -> METRY
//
// Karta katalogowa obiecuje, że napięcie jest liniowe względem ODWROTNOŚCI
// odległości (V = a/L + b). Dla NASZEGO egzemplarza to nieprawda - zmierzone
// 2026-08-20 na 7 punktach 1,05-3,69 m. Nachylenie dV/d(1/L) liczone między
// sąsiednimi punktami spada monotonicznie 167,8 -> 147,9 -> 140,2 -> 102,1 ->
// 91,0 -> 50,2, a w modelu hiperbolicznym byłoby stałe. Dopasowanie a/b myli
// się o 26 cm w walidacji leave-one-out; wariant trójparametrowy V = a/(L+d0)+b
// też nie pomaga, bo d0 wędruje -21 -> -32 -> -43 cm w miarę dokładania dalszych
// punktów, czyli pochłania krzywiznę zamiast być stałą fizyczną.
//
// Dlatego kalibracja to TABELA (calib_table), a nie para stałych, a między
// węzłami interpoluje PCHIP - monotoniczna interpolacja sześcienna
// Fritscha-Carlsona. Wybór formy jest celowo skromny: o czujniku wiemy tylko
// tyle, że wskazanie jest monotoniczne, więc interpolator, który z konstrukcji
// nie może wprowadzić niemonotoniczności ani przestrzelenia między węzłami,
// zakłada dokładnie tyle, ile wiemy. W walidacji leave-one-out daje 4,5 cm
// wobec 26,1 cm dla hiperboli.
//
// Interpolujemy 1/L, a nie L: tam zależność jest prawie liniowa i interpolator
// wnosi tylko drobną korektę zamiast odtwarzać cały kształt krzywej.
//
// GRANICE. Dwa różne powody, żeby zwrócić NaN zamiast liczby:
//   - poniżej ~100 cm krzywa Sharpa się ZAWIJA (40 cm daje to samo napięcie co
//     ~150 cm) i czujnik nie ma jak tego zasygnalizować;
//   - powyżej ~3 m czujnikowi kończy się ROZDZIELCZOŚĆ: przy 2,93 m jedno
//     zliczenie ADC to już 6 cm, przy 3,2 m ponad 10 cm, a 90% całego zakresu
//     ADC zużywa się na pierwsze 1,5 m. Odczyt stamtąd wygląda tak samo pewnie
//     jak każdy inny i nie niesie odległości.
// Poza tabelą i poza [min_range, max_range] raportujemy więc NaN. Cichy błąd
// jest przy rekonstrukcji chmury najgorszy, bo wygląda jak poprawny punkt.
class LaserSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void poll_serial(const rclcpp::Time & time);
  void process_line(const std::string & line, const rclcpp::Time & time);
  double adc_to_range(double adc) const;

  // Parsuje "ADC:metry ADC:metry ..." do calib_adc_/calib_inv_l_ i liczy
  // nachylenia PCHIP. Zwraca false przy węzłach niesparsowanych, nieposortowanych
  // albo gdy jest ich mniej niż 2.
  bool load_calib_table(const std::string & spec);
  void compute_pchip_slopes();
  double interpolate_inv_l(double adc) const;

  double get_param(const std::string & name, double fallback) const;
  std::string get_param_str(const std::string & name, const std::string & fallback) const;

  std::string sensor_name_;
  std::string device_port_;
  int baud_rate_{115200};
  int fd_{-1};

  // Tabela kalibracyjna: węzły posortowane rosnąco po ADC, wartości to 1/L
  // wyrażone w 1/cm, oraz nachylenia PCHIP policzone raz w on_init.
  // Puste = brak kalibracji, wtedy działa ścieżka zapasowa calib_a/calib_b.
  std::vector<double> calib_adc_;
  std::vector<double> calib_inv_l_;
  std::vector<double> calib_slope_;

  // Ścieżka ZAPASOWA, używana tylko gdy calib_table jest puste - żeby stack
  // wstał bez kalibracji zamiast paść. Te wartości to DWA PUNKTY Z KARTY
  // KATALOGOWEJ (100 cm -> 2,50 V, 550 cm -> 1,40 V), nie pomiar tego
  // egzemplarza, i mylą się o kilkanaście cm. Do rekonstrukcji podaj tabelę.
  double calib_a_{134.44};
  double calib_b_{1.1556};
  double vcc_{5.0};
  double adc_max_{1023.0};
  double min_range_{1.0};
  // 3,0 m, a nie 5,5 m z karty katalogowej: powyżej tego czujnikowi kończy się
  // rozdzielczość (patrz komentarz na górze). 5,5 m nigdy nie było zweryfikowane.
  double max_range_{3.0};

  // Otwarcie portu szarpie DTR, co RESETUJE Nano - przez pierwsze ~2 s leci
  // z niego cisza albo ogryzek ramki. Bez tego pierwsze odczyty po starcie
  // są śmieciowe.
  int settle_ms_{2000};

  std::string line_buffer_;

  double state_range_{0.0};
  double state_adc_{0.0};
  double state_adc_spread_{0.0};
  // Czas dotarcia ostatniej KOMPLETNEJ ramki, w sekundach zegara controller_managera.
  // Broadcaster używa go i jako znacznika czasu wiadomości, i jako testu świeżości:
  // czujnik daje realnie ~18,6 Hz, a pętla ros2_control kręci się 50 Hz, więc bez
  // tego 2/3 publikowanych pomiarów byłoby duplikatami poprzedniego odczytu.
  double state_sample_time_{0.0};

  rclcpp::Logger logger_{rclcpp::get_logger("LaserSensor")};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__LASER_SENSOR_HPP_
