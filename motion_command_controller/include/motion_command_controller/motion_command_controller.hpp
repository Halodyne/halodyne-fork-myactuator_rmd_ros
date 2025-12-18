#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace motion_command_controller {

class MotionCommandController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void command_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string joint_name_;
  std::array<double, 5> cmd_{0.0, 0.0, 0.0, 0.0, 0.0};
  std::mutex mtx_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

} // namespace motion_command_controller
