#ifndef HARDWARE_CONTROLLER__TEMPERATURE_BROADCASTER_HPP_
#define HARDWARE_CONTROLLER__TEMPERATURE_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace hardware_controller
{

// Broadcaster generyczny dla state_interface "<sensor_name>/water_temperature" -
// publikuje sensor_msgs/msg/Temperature. Odpowiednik range_sensor_broadcaster,
// ale dla temperatury, dla której ros2_controllers nie ma gotowego brokera.
class TemperatureBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string sensor_name_;
  std::string frame_id_;
  double variance_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>> rt_publisher_;
};

}  // namespace hardware_controller

#endif  // HARDWARE_CONTROLLER__TEMPERATURE_BROADCASTER_HPP_
