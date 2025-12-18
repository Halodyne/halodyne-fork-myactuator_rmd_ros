#include "motion_command_controller/motion_command_controller.hpp"

#include <algorithm>
#include <memory>

#include <pluginlib/class_list_macros.hpp>

namespace motion_command_controller {

controller_interface::CallbackReturn MotionCommandController::on_init() {
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MotionCommandController"), "No node available in on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Declare parameters similar to forward_command_controller
  // node->declare_parameter<std::vector<std::string>>("joints", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionCommandController::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  auto joints = node->get_parameter("joints").as_string_array();
  if (joints.size() != 1) {
    RCLCPP_ERROR(node->get_logger(), "Expected exactly one joint, got %zu", joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_name_ = joints.front();

  sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/command", rclcpp::QoS(10),
      std::bind(&MotionCommandController::command_cb, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "Configured MotionCommandController for joint '%s'", joint_name_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotionCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::string name = joint_name_;
  // In case joint_name_ not yet set, try to read directly from parameter
  if (name.empty()) {
    auto node = get_node();
    if (node) {
      auto joints = node->get_parameter("joints").as_string_array();
      if (!joints.empty()) {
        name = joints.front();
      }
    }
  }
  conf.names.reserve(5);
  conf.names.emplace_back(name + "/motion_p");
  conf.names.emplace_back(name + "/motion_v");
  conf.names.emplace_back(name + "/motion_kp");
  conf.names.emplace_back(name + "/motion_kd");
  conf.names.emplace_back(name + "/motion_tff");
  return conf;
}

controller_interface::InterfaceConfiguration MotionCommandController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::return_type MotionCommandController::update(
    const rclcpp::Time &, const rclcpp::Duration &) {
  std::array<double, 5> local{};
  {
    std::lock_guard<std::mutex> lk(mtx_);
    local = cmd_;
  }
  if (command_interfaces_.size() != 5) {
    return controller_interface::return_type::ERROR;
  }
  // write values into the five claimed interfaces
  for (std::size_t i = 0; i < 5; ++i) {
    command_interfaces_[i].set_value(local[i]);
  }
  return controller_interface::return_type::OK;
}

void MotionCommandController::command_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!msg || msg->data.size() < 5) {
    return; // ignore
  }
  std::array<double, 5> next{
      msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4]};
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_ = next;
  }
}

}  // namespace motion_command_controller

PLUGINLIB_EXPORT_CLASS(motion_command_controller::MotionCommandController, controller_interface::ControllerInterface)
