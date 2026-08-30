#include "hardware_controller/laser_broadcaster.hpp"

#include <cstdint>

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

controller_interface::CallbackReturn LaserBroadcaster::on_init()
{
  try
  {
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("frame_id", "");
    auto_declare<double>("field_of_view", 0.0);
    auto_declare<double>("min_range", 1.0);
    auto_declare<double>("max_range", 5.5);
    auto_declare<int>("radiation_type", 1);
    // Dwa interfejsy diagnostyczne trafiające na ~/raw jako x i y. Nazwy są
    // parametrem, bo zależą od czujnika: Sharp dawał surowe ADC i jego rozrzut,
    // JRT daje jakość sygnału i kod statusu. Wcześniej były wpisane na sztywno,
    // przez co ten sam broadcaster nie dawał się użyć do obu.
    // TRZY nazwy: trafiają na ~/raw jako x, y, z. Zależą od czujnika - Sharp
    // dawał surowe ADC i jego rozrzut, dalmierz JRT jakość sygnału, kod statusu
    // i MOMENT ROZPOCZĘCIA STRZAŁU. Ten ostatni jest tu najważniejszy: pozwala
    // odbiorcy odrzucić pomiar, który zaczął się jeszcze przy ruchomej głowicy.
    // Domyślne "adc,adc_spread,sample_time" odtwarza dokładnie poprzednie
    // zachowanie, gdzie z było powtórzonym stemplem.
    auto_declare<std::string>("diag_interfaces", "adc,adc_spread,sample_time");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Wyjątek w on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LaserBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration LaserBroadcaster::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    // Kolejność jest kontraktem z update(), które indeksuje state_interfaces_
    // po pozycji: range=0, sample_time=1, diag0=2, diag1=3. Dokładać wolno
    // tylko na końcu.
    {sensor_name_ + "/range", sensor_name_ + "/sample_time",
     sensor_name_ + "/" + diag0_, sensor_name_ + "/" + diag1_,
     sensor_name_ + "/" + diag2_}};
}

controller_interface::CallbackReturn LaserBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  field_of_view_ = get_node()->get_parameter("field_of_view").as_double();
  min_range_ = get_node()->get_parameter("min_range").as_double();
  max_range_ = get_node()->get_parameter("max_range").as_double();
  radiation_type_ = static_cast<int>(get_node()->get_parameter("radiation_type").as_int());

  const std::string diag = get_node()->get_parameter("diag_interfaces").as_string();
  std::vector<std::string> names;
  size_t start = 0;
  while (true)
  {
    const size_t comma = diag.find(',', start);
    names.push_back(diag.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    if (comma == std::string::npos) { break; }
    start = comma + 1;
  }
  if (names.size() != 3 || names[0].empty() || names[1].empty() || names[2].empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'diag_interfaces' musi mieć postać '<nazwa>,<nazwa>,<nazwa>', jest '%s'", diag.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  diag0_ = names[0];
  diag1_ = names[1];
  diag2_ = names[2];

  if (sensor_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (frame_id_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::Range>(
    "~/range", rclcpp::SystemDefaultsQoS());
  rt_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Range>>(publisher_);

  rt_publisher_->lock();
  rt_publisher_->msg_.header.frame_id = frame_id_;
  rt_publisher_->msg_.radiation_type = static_cast<uint8_t>(radiation_type_);
  rt_publisher_->msg_.field_of_view = static_cast<float>(field_of_view_);
  rt_publisher_->msg_.min_range = static_cast<float>(min_range_);
  rt_publisher_->msg_.max_range = static_cast<float>(max_range_);
  rt_publisher_->unlock();

  // Surowe ADC obok metrów - znaczenie pól opisane w nagłówku klasy.
  raw_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/raw", rclcpp::SystemDefaultsQoS());
  rt_raw_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(
      raw_publisher_);

  rt_raw_publisher_->lock();
  rt_raw_publisher_->msg_.header.frame_id = frame_id_;
  rt_raw_publisher_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LaserBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  last_sample_time_ = 0.0;
  last_raw_sample_time_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LaserBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LaserBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Kolejność interfejsów jest ta sama, co w state_interface_configuration().
  const double range = state_interfaces_[0].get_value();
  const double sample_time = state_interfaces_[1].get_value();
  const double diag0 = state_interfaces_[2].get_value();
  const double diag1 = state_interfaces_[3].get_value();
  const double diag2 = state_interfaces_[4].get_value();

  // Zero = wtyczka nie dostała jeszcze ani jednej kompletnej ramki.
  if (sample_time <= 0.0)
  {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Time stamp(static_cast<int64_t>(sample_time * 1e9), RCL_ROS_TIME);

  if (sample_time != last_sample_time_ && rt_publisher_ && rt_publisher_->trylock())
  {
    rt_publisher_->msg_.header.stamp = stamp;
    rt_publisher_->msg_.range = static_cast<float>(range);
    rt_publisher_->unlockAndPublish();
    last_sample_time_ = sample_time;
  }

  // Ten sam stempel co w Range - to jest klucz, po którym kolektor paruje
  // surowy odczyt z punktem chmury. Range publikuje się nawet przy NaN
  // (odczyt poza wiarygodnym zakresem), więc raw też, bo właśnie te punkty
  // najbardziej interesują przy sprawdzaniu innej kalibracji.
  //
  // KAŻDY topic ma WŁASNY licznik ostatniej opublikowanej próbki i to nie jest
  // nadmiarowa ostrożność. Przy wspólnym liczniku nieudany trylock raw gubił
  // ramkę BEZPOWROTNIE: Range zdążył podbić licznik, więc w następnym cyklu
  // warunek "to już publikowaliśmy" wycinał całą funkcję, zanim doszła do raw.
  // Objaw: rosnący licznik "bez surowego ADC" w kolektorze, tym szybciej, im
  // bardziej obciążona płytka (trylock nie wchodzi, gdy wątek publikujący
  // jeszcze nie oddał poprzedniej wiadomości) - zmierzone na sweepie
  // 2026-08-21: 6 zgubionych przez pierwszą godzinę, potem 6 w kilka minut.
  // Osobny licznik sprawia, że nieudana próba wraca po 20 ms, czyli grubo
  // przed następną ramką czujnika (~50 ms).
  if (sample_time != last_raw_sample_time_ && rt_raw_publisher_ && rt_raw_publisher_->trylock())
  {
    rt_raw_publisher_->msg_.header.stamp = stamp;
    rt_raw_publisher_->msg_.vector.x = diag0;
    rt_raw_publisher_->msg_.vector.y = diag1;
    rt_raw_publisher_->msg_.vector.z = diag2;
    rt_raw_publisher_->unlockAndPublish();
    last_raw_sample_time_ = sample_time;
  }

  return controller_interface::return_type::OK;
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::LaserBroadcaster, controller_interface::ControllerInterface)
