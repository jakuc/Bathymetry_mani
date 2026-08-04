#ifndef HARDWARE_CONTROLLER__GNSS_SENSOR_HPP_
#define HARDWARE_CONTROLLER__GNSS_SENSOR_HPP_

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

// Odbiornik GNSS Septentrio mosaic-H (dwuantenowy — pozycja + heading):
// NMEA 0183 po USB CDC-ACM (baud rate bez znaczenia dla CDC, ale ustawiany
// dla spójności z resztą wtyczek). Wtyczka przy on_configure SAMA konfiguruje
// odbiornik komendą Septentrio (setNMEAOutput ... GGA+GST+HDT), więc nie
// wymaga wcześniejszego klikania w web UI odbiornika, a przy on_deactivate
// wyłącza strumień, zostawiając odbiornik w stanie zastanym.
//
// Zdania potwierdzone na żywym mosaic-H (fw 4.14.0):
//   $GPGGA — lat/lon/alt(MSL)+separacja geoidy, jakość fixa, liczba satelitów,
//            a przy RTK dodatkowo wiek korekcji (pole 13) i ID stacji (pole 14)
//   $GPGST — odchylenia standardowe lat/lon/alt (m) → kowariancja NavSatFix
//   $GPHDT — heading rzeczywisty (deg) z bazy dwóch anten
//   $PSSN,SNC — proprietary Septentrio: status klienta NTRIP (RTK). Bez tego
//            "brak RTK" jest nierozróżnialny od "caster odrzucił hasło" — a to
//            jedyne, co widać w terenie, gdy fix nie wchodzi w 4.
// Puste pola (brak fixa) → NaN. Checksum nie jest weryfikowany — transport to
// USB CDC z własnym CRC na poziomie USB, uszkodzone ramki nie występują
// (spójne z EchosounderSensor).
class GnssSensor : public hardware_interface::SensorInterface
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
  void send_receiver_command(const std::string & command);
  // Czeka na echo "$R:" + prompt, żeby kolejna komenda nie przepadła.
  void wait_for_command_ack(const std::string & command);
  // Loguje zmianę stanu klienta NTRIP słownie (kody z NTRIPClientStatus).
  void log_ntrip_transition();

  std::string sensor_name_;
  std::string device_port_;
  int baud_rate_{115200};
  // Deskryptor portu po stronie odbiornika (USB1 = pierwszy port CDC) oraz
  // numer strumienia NMEA i tempo — parametry komendy setNMEAOutput.
  std::string receiver_port_{"USB1"};
  int nmea_stream_{1};
  std::string nmea_rate_{"msec100"};
  // Status NTRIP idzie osobnym strumieniem NMEA, wolniej niż pozycja: przy
  // msec100 dokładałby 10 zdań/s czystej diagnostyki. Ustaw 0, żeby wyłączyć.
  // UWAGA: strumienie NMEA mają wspólną pulę (Stream1..Stream10) — skrypty
  // diagnostyczne na /dev/gnss_aux muszą używać numeru spoza {nmea, status}.
  int status_stream_{2};
  std::string status_rate_{"sec1"};
  // Który deskryptor połączenia NTRIP obserwujemy (1 = NTR1). Odbiornik
  // raportuje w SNC wszystkie naraz, więc trzeba wybrać po CDIndex.
  int ntrip_connection_{1};
  int fd_{-1};

  std::string line_buffer_;
  double state_latitude_{0.0};        // deg, +N/-S
  double state_longitude_{0.0};       // deg, +E/-W
  double state_altitude_{0.0};        // m, wysokość elipsoidalna (MSL + separacja geoidy z GGA)
  double state_fix_quality_{0.0};     // GGA: 0=brak, 1=GPS, 2=DGPS, 4=RTK fix, 5=RTK float
  double state_num_satellites_{0.0};
  double state_lat_std_{0.0};         // m (GST)
  double state_lon_std_{0.0};         // m (GST)
  double state_alt_std_{0.0};         // m (GST)
  double state_heading_{0.0};         // deg, true north, CW (HDT z dwóch anten)
  // RTK: wiek korekcji różnicowych (s) i ID stacji bazowej / VRS z GGA.
  // Wiek to najczulszy wskaźnik zdrowia łącza — rośnie w sekundach od chwili,
  // gdy strumień NTRIP przestaje płynąć, jeszcze zanim fix spadnie z 4 na 1.
  double state_diff_age_{0.0};        // s (GGA pole 13), NaN gdy brak korekcji
  double state_station_id_{0.0};      // GGA pole 14, NaN gdy brak korekcji
  // Status klienta NTRIP z $PSSN,SNC (bloki NTRIPClientStatus):
  //   status: 0=wyłączony, 1=inicjalizacja, 2=działa, 3=błąd, 4=ponawianie, 5=duplikat
  //   error:  0=brak, 1=init, 2=autoryzacja, 3=połączenie, 4=brak mountpointu,
  //           5=mountpoint niedostępny, 6=czeka na GGA, 7=GGA wyłączone,
  //           8=DNS, 9=poza zasięgiem, 10-13=TLS, 254=nieznany
  double state_ntrip_status_{0.0};
  double state_ntrip_error_{0.0};
  // Licznik sparsowanych ramek GGA. Broadcaster publikuje tylko gdy licznik
  // się zmienił — dzięki temu "brak fixa" (NaN @ 10 Hz na topiku) da się
  // odróżnić od "urządzenie nie gada" (topik milczy). Bez tego oba przypadki
  // wyglądają identycznie: NaN powtarzane w takcie update_rate.
  double state_data_count_{0.0};
  // Ostatnio zalogowany stan NTRIP — żeby logować przejścia, nie co sekundę.
  int last_logged_ntrip_status_{-1};
  int last_logged_ntrip_error_{-1};

  rclcpp::Logger logger_{rclcpp::get_logger("GnssSensor")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__GNSS_SENSOR_HPP_
