#include "hardware_controller/temperature_broadcaster.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace hardware_controller
{

controller_interface::CallbackReturn TemperatureBroadcaster::on_init()
{
  try
  {
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("frame_id", "");
    auto_declare<double>("variance", 0.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Wyjątek w on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TemperatureBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration TemperatureBroadcaster::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {sensor_name_ + "/water_temperature"}};
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  variance_ = get_node()->get_parameter("variance").as_double();

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

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::Temperature>(
    "~/temperature", rclcpp::SystemDefaultsQoS());
  rt_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>>(publisher_);

  rt_publisher_->lock();
  rt_publisher_->msg_.header.frame_id = frame_id_;
  rt_publisher_->msg_.variance = variance_;
  rt_publisher_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TemperatureBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (rt_publisher_ && rt_publisher_->trylock())
  {
    rt_publisher_->msg_.header.stamp = time;
    rt_publisher_->msg_.temperature = state_interfaces_[0].get_value();
    rt_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace hardware_controller

PLUGINLIB_EXPORT_CLASS(hardware_controller::TemperatureBroadcaster, controller_interface::ControllerInterface)
