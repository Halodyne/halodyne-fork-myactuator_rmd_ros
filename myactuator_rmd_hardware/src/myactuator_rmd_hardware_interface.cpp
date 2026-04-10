#include "myactuator_rmd_hardware/myactuator_rmd_hardware_interface.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<pthread.h>) && __has_include(<sched.h>)
  #include <pthread.h>
  #include <sched.h>
  #define MYACTUATOR_RMD_HARDWARE__THREAD_PRIORITY
#endif

#include <myactuator_rmd/can/exceptions.hpp>
#include <myactuator_rmd/exceptions.hpp>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>


#include "myactuator_rmd_hardware/conversions.hpp"
#include "myactuator_rmd_hardware/low_pass_filter.hpp"
#include "myactuator_rmd_hardware/velocity_estimators.hpp"


namespace myactuator_rmd_hardware {

  using CallbackReturn = MyActuatorRmdHardwareInterface::CallbackReturn;

  MyActuatorRmdHardwareInterface::~MyActuatorRmdHardwareInterface() {
    // If the controller manager is shutdown with Ctrl + C the on_deactivate methods won't be called!
    // We therefore shut down the actuator here.
    on_cleanup(rclcpp_lifecycle::State());
    return;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_configure(rclcpp_lifecycle::State const& /*previous_state*/) {
    if (info_.hardware_parameters.find("ifname") != info_.hardware_parameters.end()) {
      ifname_ = info_.hardware_parameters["ifname"];
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN interface name!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("actuator_id") != info_.hardware_parameters.end()) {
      actuator_id_ = std::stoi(info_.hardware_parameters["actuator_id"]);
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN actuator id!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("torque_constant") != info_.hardware_parameters.end()) {
      torque_constant_ = std::stod(info_.hardware_parameters["torque_constant"]);
    } else {
      torque_constant_ = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(getLogger(), "Could not parse torque constant, won't be able to use torque interface!");
    }
    if (info_.hardware_parameters.find("max_velocity") != info_.hardware_parameters.end()) {
      max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
    } else {
      max_velocity_ = 720.0;
      RCLCPP_INFO(getLogger(), "Max velocity not set, defaulting to '%f'.", max_velocity_);
    }
    if (info_.hardware_parameters.find("velocity_alpha") != info_.hardware_parameters.end()) {
      auto const velocity_alpha {std::stod(info_.hardware_parameters["velocity_alpha"])};
      velocity_low_pass_filter_ = std::make_unique<LowPassFilter>(velocity_alpha);
      RCLCPP_INFO(getLogger(), "Using velocity low-pass filter with filter constant '%f'.", velocity_alpha);
    } else {
      velocity_low_pass_filter_ = nullptr;
      RCLCPP_INFO(getLogger(), "Not using velocity low-pass filter.");
    }
    if (info_.hardware_parameters.find("effort_alpha") != info_.hardware_parameters.end()) {
      auto const effort_alpha {std::stod(info_.hardware_parameters["effort_alpha"])};
      effort_low_pass_filter_ = std::make_unique<LowPassFilter>(effort_alpha);
      RCLCPP_INFO(getLogger(), "Using effort low-pass filter with filter constant '%f'.", effort_alpha);
    } else {
      effort_low_pass_filter_ = nullptr;
      RCLCPP_INFO(getLogger(), "Not using effort low-pass filter.");
    }
    if (info_.hardware_parameters.find("cycle_time") != info_.hardware_parameters.end()) {
      cycle_time_ = std::chrono::milliseconds(std::stol(info_.hardware_parameters["cycle_time"]));
    } else {
      cycle_time_ = std::chrono::milliseconds(2);
      RCLCPP_INFO(getLogger(), "Cycle time not set, defaulting to '%ld' ms.", cycle_time_.count());
    }
    if (info_.hardware_parameters.find("timeout") != info_.hardware_parameters.end()) {
      timeout_= std::chrono::milliseconds(std::stoi(info_.hardware_parameters["timeout"]));
    } else {
      timeout_ = std::chrono::milliseconds(0);
      RCLCPP_INFO(getLogger(), "Timeout not set, defaulting to '%ld' ms", timeout_.count());
    }
    if (timeout_ == std::chrono::milliseconds(0)) {
      RCLCPP_INFO(getLogger(), "Timeout set to 0ms, it will not be used!");
    }
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    actuator_interface_ = std::make_unique<myactuator_rmd::ActuatorInterface>(*driver_, actuator_id_);
    motion_driver_ = std::make_unique<myactuator_rmd::motion_mode::CanDriver>(ifname_);
    motion_actuator_interface_ = std::make_unique<myactuator_rmd::motion_mode::ActuatorInterface>(*motion_driver_, actuator_id_);
    if (!actuator_interface_) {
      RCLCPP_INFO(getLogger(), "Failed to create actuator interface!");
      return CallbackReturn::ERROR;
    }
    if (!motion_actuator_interface_) {
      RCLCPP_INFO(getLogger(), "Failed to create motion mode actuator interface!");
      return CallbackReturn::ERROR;
    }
    try {
      std::string const motor_model {actuator_interface_->getMotorModel()};
      RCLCPP_INFO(getLogger(), "Started actuator interface for actuator model '%s'!", motor_model.c_str());
      motor_online_.store(true);
    } catch (const std::exception& e) {
      RCLCPP_WARN(getLogger(), "Motor %u not responding at startup (%s), will probe in background",
        actuator_id_, e.what());
      motor_online_.store(false);
      motor_first_reading_valid_.store(false);
      async_position_state_.store(std::numeric_limits<double>::quiet_NaN());
      async_velocity_state_.store(std::numeric_limits<double>::quiet_NaN());
      async_effort_state_.store(std::numeric_limits<double>::quiet_NaN());
    }
    // Parse 1-Euro filter parameters from URDF hardware parameters
    double oe_min_cutoff = 1.0;
    double oe_beta = 5.0;
    double oe_d_cutoff = 1.0;
    if (info_.hardware_parameters.find("velocity_min_cutoff") != info_.hardware_parameters.end()) {
      oe_min_cutoff = std::stod(info_.hardware_parameters["velocity_min_cutoff"]);
    }
    if (info_.hardware_parameters.find("velocity_beta") != info_.hardware_parameters.end()) {
      oe_beta = std::stod(info_.hardware_parameters["velocity_beta"]);
    }
    if (info_.hardware_parameters.find("velocity_d_cutoff") != info_.hardware_parameters.end()) {
      oe_d_cutoff = std::stod(info_.hardware_parameters["velocity_d_cutoff"]);
    }
    RCLCPP_INFO(getLogger(), "1-Euro velocity filter: min_cutoff=%.2f Hz, beta=%.2f, d_cutoff=%.2f Hz",
      oe_min_cutoff, oe_beta, oe_d_cutoff);

    // Create 1-Euro velocity estimator
    double const dt_s = cycle_time_.count() / 1000.0;
    velocity_estimator_ = std::make_unique<OneEuroEstimator>(dt_s, oe_min_cutoff, oe_beta, oe_d_cutoff);

    // Create parameter node for runtime tuning
    auto const id_str = std::to_string(actuator_id_);
    param_node_ = rclcpp::Node::make_shared("motor_" + id_str + "_params");
    motion_debug_pub_ = param_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "motor_" + id_str + "_motion_raw", 10);
    param_node_->declare_parameter("velocity_min_cutoff", oe_min_cutoff);
    param_node_->declare_parameter("velocity_beta", oe_beta);
    param_node_->declare_parameter("velocity_d_cutoff", oe_d_cutoff);
    param_callback_handle_ = param_node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        for (auto const& p : params) {
          if (p.get_name() == "velocity_min_cutoff") {
            velocity_estimator_->setMinCutoff(p.as_double());
            RCLCPP_INFO(getLogger(), "Motor %u velocity_min_cutoff set to %.2f", actuator_id_, p.as_double());
          } else if (p.get_name() == "velocity_beta") {
            velocity_estimator_->setBeta(p.as_double());
            RCLCPP_INFO(getLogger(), "Motor %u velocity_beta set to %.2f", actuator_id_, p.as_double());
          } else if (p.get_name() == "velocity_d_cutoff") {
            velocity_estimator_->setDCutoff(p.as_double());
            RCLCPP_INFO(getLogger(), "Motor %u velocity_d_cutoff set to %.2f", actuator_id_, p.as_double());
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // Spin param node in background thread for runtime parameter updates
    param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    param_executor_->add_node(param_node_);
    param_spin_thread_ = std::thread([this]() {
      param_executor_->spin();
    });

    stop_async_thread_.store(false);
    if (!startAsyncThread(cycle_time_)) {
      RCLCPP_FATAL(getLogger(), "Failed to start async thread!");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_cleanup(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (param_executor_) {
      param_executor_->cancel();
    }
    if (param_spin_thread_.joinable()) {
      param_spin_thread_.join();
    }
    if (actuator_interface_) {
      actuator_interface_->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_shutdown(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (param_executor_) {
      param_executor_->cancel();
    }
    if (param_spin_thread_.joinable()) {
      param_spin_thread_.join();
    }
    if (actuator_interface_) {
      actuator_interface_->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_activate(rclcpp_lifecycle::State const& /*previous_state*/) {
    RCLCPP_INFO(getLogger(), "Actuator successfully started!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_deactivate(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      actuator_interface_->stopMotor();
    }
    RCLCPP_INFO(getLogger(), "Actuator successfully stopped!");
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MyActuatorRmdHardwareInterface::on_error(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      actuator_interface_->stopMotor();
      actuator_interface_->reset();
      RCLCPP_INFO(getLogger(), "Actuator reset!");
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_init(hardware_interface::HardwareInfo const& info) {
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    position_state_ = 0.0;
    velocity_state_ = 0.0;
    effort_state_ = 0.0;
    position_command_ = 0.0;
    velocity_command_ = 0.0;
    effort_command_ = 0.0;
    motion_p_command_ = 0.0;
    motion_v_command_ = 0.0;
    motion_kp_command_ = 0.0;
    motion_kd_command_ = 0.0;
    motion_tff_command_ = 0.0;

    position_interface_running_.store(false);
    velocity_interface_running_.store(false);
    effort_interface_running_.store(false);
    motion_interface_running_.store(false);

    if (info_.joints.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Expected a single joint but got %zu joints.", info_.joints.size());
      return CallbackReturn::ERROR;
    }
    hardware_interface::ComponentInfo const& joint = info_.joints.at(0);
    // Accept at least the standard three command interfaces; allow extra motion_* ones
    bool has_pos{false}, has_vel{false}, has_eff{false};
    for (auto const &ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) has_pos = true;
      else if (ci.name == hardware_interface::HW_IF_VELOCITY) has_vel = true;
      else if (ci.name == hardware_interface::HW_IF_EFFORT) has_eff = true;
      // allow custom motion_* interfaces if present; no strict check here
    }
    if (!(has_pos && has_vel && has_eff)) {
      RCLCPP_FATAL(getLogger(),
        "Joint '%s' must expose position, velocity, and effort command interfaces.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT
      );
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorRmdHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces {};
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_state_)
    );
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MyActuatorRmdHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces {};
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_command_)
    );
    // Optional motion mode command interfaces (position, velocity, kp, kd, torque_ff)
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, "motion_p", &motion_p_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, "motion_v", &motion_v_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, "motion_kp", &motion_kp_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, "motion_kd", &motion_kd_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, "motion_tff", &motion_tff_command_)
    );
    return command_interfaces;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::prepare_command_mode_switch(
    std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
    bool position_interface_claimed {position_interface_running_.load()};
    bool velocity_interface_claimed {velocity_interface_running_.load()};
    bool effort_interface_claimed {effort_interface_running_.load()};
    bool motion_interface_claimed {motion_interface_running_.load()};

    for (auto const& stop_interface: stop_interfaces) {
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/motion_p" ||
                 stop_interface == info_.joints.at(0).name + "/motion_v" ||
                 stop_interface == info_.joints.at(0).name + "/motion_kp" ||
                 stop_interface == info_.joints.at(0).name + "/motion_kd" ||
                 stop_interface == info_.joints.at(0).name + "/motion_tff") {
        motion_interface_claimed = false;
      }
    }

    // Count motion_* interfaces that are requested to start
    std::size_t motion_start_count {0};
    for (auto const &start_interface : start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/motion_p" ||
          start_interface == info_.joints.at(0).name + "/motion_v" ||
          start_interface == info_.joints.at(0).name + "/motion_kp" ||
          start_interface == info_.joints.at(0).name + "/motion_kd" ||
          start_interface == info_.joints.at(0).name + "/motion_tff") {
        ++motion_start_count;
      }
    }

    for (auto const& start_interface: start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        if (velocity_interface_claimed || effort_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim position interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        if (motion_interface_claimed || motion_start_count > 0) {
          RCLCPP_ERROR(getLogger(), "Can't claim position interface: Conflicts with motion interfaces!");
          return hardware_interface::return_type::ERROR;
        }
        position_interface_claimed = true;
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (position_interface_claimed || effort_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim velocity interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        if (motion_interface_claimed || motion_start_count > 0) {
          RCLCPP_ERROR(getLogger(), "Can't claim velocity interface: Conflicts with motion interfaces!");
          return hardware_interface::return_type::ERROR;
        }
        velocity_interface_claimed = true;
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        if (std::isnan(torque_constant_)) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Invalid torque constant!");
          return hardware_interface::return_type::ERROR;
        } else if (position_interface_claimed || velocity_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        if (motion_interface_claimed || motion_start_count > 0) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Conflicts with motion interfaces!");
          return hardware_interface::return_type::ERROR;
        }
        effort_interface_claimed = true;
      }
    }

    if (motion_start_count > 0) {
      if (motion_start_count != 5) {
        RCLCPP_ERROR(getLogger(), "Must claim all 5 motion_* interfaces together (p, v, kp, kd, tff)!");
        return hardware_interface::return_type::ERROR;
      }
      if (position_interface_claimed || velocity_interface_claimed || effort_interface_claimed) {
        RCLCPP_ERROR(getLogger(), "Can't claim motion interfaces: Conflicting standard interface claimed!");
        return hardware_interface::return_type::ERROR;
      }
      motion_interface_claimed = true;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::perform_command_mode_switch(
    std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
    for (auto const& stop_interface: stop_interfaces) {
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping position interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping velocity interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping effort interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/motion_p" ||
                 stop_interface == info_.joints.at(0).name + "/motion_v" ||
                 stop_interface == info_.joints.at(0).name + "/motion_kp" ||
                 stop_interface == info_.joints.at(0).name + "/motion_kd" ||
                 stop_interface == info_.joints.at(0).name + "/motion_tff") {
        motion_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping motion interfaces...");
      }
    }

    std::size_t motion_start_count {0};
    for (auto const &start_interface : start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/motion_p" ||
          start_interface == info_.joints.at(0).name + "/motion_v" ||
          start_interface == info_.joints.at(0).name + "/motion_kp" ||
          start_interface == info_.joints.at(0).name + "/motion_kd" ||
          start_interface == info_.joints.at(0).name + "/motion_tff") {
        ++motion_start_count;
      }
    }

    for (auto const& start_interface: start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting position interface...");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting velocity interface...");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting effort interface...");
      }
    }

    if (motion_start_count == 5) {
      motion_interface_running_.store(true);
      RCLCPP_INFO(getLogger(), "Starting motion interfaces...");
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::read(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    position_state_ = async_position_state_.load();
    velocity_state_ = async_velocity_state_.load();
    effort_state_ = async_effort_state_.load();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::write(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    // TODO: Make sure that all commands are finite
    if (position_interface_running_) {
      async_position_command_.store(position_command_);
    } else if (velocity_interface_running_) {
      async_velocity_command_.store(velocity_command_);
    } else if (effort_interface_running_) {
      async_effort_command_.store(effort_command_);
    } else if (motion_interface_running_) {
      async_motion_p_command_.store(motion_p_command_);
      async_motion_v_command_.store(motion_v_command_);
      async_motion_kp_command_.store(motion_kp_command_);
      async_motion_kd_command_.store(motion_kd_command_);
      async_motion_tff_command_.store(motion_tff_command_);
    }
    return hardware_interface::return_type::OK;
  }

  rclcpp::Logger MyActuatorRmdHardwareInterface::getLogger() {
    return rclcpp::get_logger("MyActuatorRmdHardwareInterface");
  }

  void MyActuatorRmdHardwareInterface::asyncThread(std::chrono::milliseconds const& cycle_time) {
    if (motor_online_.load()) {
      try {
        actuator_interface_->setTimeout(timeout_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(getLogger(), "Motor %u failed to set timeout: %s", actuator_id_, e.what());
        motor_online_.store(false);
        motor_first_reading_valid_.store(false);
      }
    }

    constexpr int MAX_CONSECUTIVE_ERRORS = 5;
    int consecutive_errors = 0;
    bool first_error_logged = false;
    int offline_tick_counter = 0;
    int const probe_interval_ticks = std::max(1, static_cast<int>(500 / cycle_time.count()));

    auto go_offline = [this, &offline_tick_counter]() {
      motor_online_.store(false);
      motor_first_reading_valid_.store(false);
      async_position_state_.store(std::numeric_limits<double>::quiet_NaN());
      async_velocity_state_.store(std::numeric_limits<double>::quiet_NaN());
      async_effort_state_.store(std::numeric_limits<double>::quiet_NaN());
      offline_tick_counter = 0;
    };

    auto handle_error = [this, &consecutive_errors, &first_error_logged, &go_offline](const std::exception& e) {
      consecutive_errors++;
      if (!first_error_logged) {
        RCLCPP_WARN(getLogger(), "Motor %u CAN error: %s", actuator_id_, e.what());
        first_error_logged = true;
      }
      if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
        RCLCPP_ERROR(getLogger(), "Motor %u went offline after %d consecutive errors",
          actuator_id_, MAX_CONSECUTIVE_ERRORS);
        go_offline();
      }
    };

#ifdef MYACTUATOR_RMD_HARDWARE__ASYNC_LOOP_STATS
    std::uint64_t loop_count {0};
    std::uint64_t overrun_count {0};
    double total_io_us {0.0};
    double max_io_us {0.0};
    auto stats_start = std::chrono::steady_clock::now();
    const auto stats_interval = std::chrono::seconds(5);
#endif

    auto wakeup_time {std::chrono::steady_clock::now()};
    while (!stop_async_thread_) {
      auto const now {std::chrono::steady_clock::now()};
      wakeup_time += cycle_time;

      // --- OFFLINE: probe periodically ---
      if (!motor_online_.load()) {
        offline_tick_counter++;
        if (offline_tick_counter >= probe_interval_ticks) {
          offline_tick_counter = 0;
          try {
            (void)actuator_interface_->getMotorModel();
            actuator_interface_->setTimeout(timeout_);
            motor_online_.store(true);
            motor_first_reading_valid_.store(false);
            consecutive_errors = 0;
            first_error_logged = false;
            RCLCPP_INFO(getLogger(), "Motor %u came online", actuator_id_);
          } catch (...) {
            // Still offline
          }
        }
        std::this_thread::sleep_until(wakeup_time);
        continue;
      }

      // --- FIRST READING: read position only, no command ---
      if (!motor_first_reading_valid_.load()) {
        try {
          feedback_ = actuator_interface_->getMotorStatus2();
          double const position_state {feedback_.shaft_angle};
          double velocity_state {feedback_.shaft_speed};
          if (velocity_low_pass_filter_) {
            velocity_state = velocity_low_pass_filter_->apply(velocity_state);
          }
          double current_state {feedback_.current};
          if (effort_low_pass_filter_) {
            current_state = effort_low_pass_filter_->apply(current_state);
          }
          async_position_state_.store(degToRad(position_state));
          async_velocity_state_.store(degToRad(velocity_state));
          async_effort_state_.store(currentToTorque(current_state, torque_constant_));
          motor_first_reading_valid_.store(true);
          RCLCPP_INFO(getLogger(), "Motor %u first reading valid (pos=%.1f deg)", actuator_id_, position_state);
        } catch (const std::exception& e) {
          RCLCPP_WARN(getLogger(), "Motor %u first reading failed: %s", actuator_id_, e.what());
          go_offline();
        }
        std::this_thread::sleep_until(wakeup_time);
        continue;
      }

      // --- NORMAL OPERATION ---
      try {
        if (position_interface_running_) {
          feedback_ = actuator_interface_->sendPositionAbsoluteSetpoint(radToDeg(async_position_command_.load()), max_velocity_);
        } else if (velocity_interface_running_) {
          feedback_ = actuator_interface_->sendVelocitySetpoint(radToDeg(async_velocity_command_.load()));
        } else if (effort_interface_running_) {
          feedback_ = actuator_interface_->sendTorqueSetpoint(async_effort_command_.load(), torque_constant_);
        } else if (motion_interface_running_) {
          // Clamp kp/kd to [0, 4095] and cast to uint16_t
          auto clamp12 = [](double x) -> std::uint16_t {
            if (x < 0.0) return static_cast<std::uint16_t>(0);
            if (x > 4095.0) return static_cast<std::uint16_t>(4095);
            return static_cast<std::uint16_t>(x);
          };
          auto const p = async_motion_p_command_.load();
          auto const v = async_motion_v_command_.load();
          auto const kp = clamp12(async_motion_kp_command_.load());
          auto const kd = clamp12(async_motion_kd_command_.load());
          auto const tff = async_motion_tff_command_.load();
          // Interleaved send/recv: send both commands first, then receive both
          // responses. This overlaps the motor processing time for the angle
          // request with the wait for the motion mode response, saving ~0.75ms
          // per cycle and enabling 500Hz operation.
          motion_actuator_interface_->sendMotionModeControl(static_cast<float>(p), static_cast<float>(v), kp, kd, static_cast<float>(tff));
          actuator_interface_->sendGetMultiTurnAngle();
          auto response = motion_actuator_interface_->recvMotionModeControl();
          auto const position = actuator_interface_->recvGetMultiTurnAngle();
#ifdef MYACTUATOR_RMD_HARDWARE__ASYNC_LOOP_STATS
          {
            auto const io_end {std::chrono::steady_clock::now()};
            double const io_us {std::chrono::duration<double, std::micro>(io_end - now).count()};
            total_io_us += io_us;
            if (io_us > max_io_us) max_io_us = io_us;
            ++loop_count;
            if (io_end > wakeup_time) ++overrun_count;
            if (io_end - stats_start >= stats_interval) {
              double const actual_rate = static_cast<double>(loop_count) /
                std::chrono::duration<double>(io_end - stats_start).count();
              RCLCPP_INFO(getLogger(),
                "Async loop [id=%u]: rate=%.1f Hz, io_mean=%.0f us, io_max=%.0f us, overruns=%lu/%lu (%.1f%%)",
                actuator_id_, actual_rate, total_io_us / loop_count, max_io_us,
                overrun_count, loop_count,
                100.0 * static_cast<double>(overrun_count) / loop_count);
              loop_count = 0;
              overrun_count = 0;
              total_io_us = 0.0;
              max_io_us = 0.0;
              stats_start = io_end;
            }
          }
#endif
          auto const position_rad = degToRad(position);
          async_position_state_.store(position_rad);
          async_effort_state_.store(response.getTorque());
          auto const velocity_rad = velocity_estimator_->update(position_rad);
          async_velocity_state_.store(velocity_rad);
          // Publish raw motion debug data: [cmd_pos, cmd_vel, raw_pos, raw_vel, torque]
          {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {p, v, position_rad, static_cast<double>(response.getVelocity()),
                        static_cast<double>(response.getTorque())};
            motion_debug_pub_->publish(msg);
          }
          feedback_.shaft_angle = position;
          feedback_.shaft_speed = radToDeg(velocity_rad);
          feedback_.current = torqueToCurrent(response.getTorque(), torque_constant_);
          consecutive_errors = 0;
          first_error_logged = false;
          std::this_thread::sleep_until(wakeup_time);
          continue;
        } else {
          feedback_ = actuator_interface_->getMotorStatus2();
        }

        double const position_state {feedback_.shaft_angle};
        double velocity_state {feedback_.shaft_speed};
        if (velocity_low_pass_filter_) {
          velocity_state = velocity_low_pass_filter_->apply(velocity_state);
        }
        double current_state {feedback_.current};
        if (effort_low_pass_filter_) {
          current_state = effort_low_pass_filter_->apply(current_state);
        }
        async_position_state_.store(degToRad(position_state));
        async_velocity_state_.store(degToRad(velocity_state));
        async_effort_state_.store(currentToTorque(current_state, torque_constant_));

        consecutive_errors = 0;
        first_error_logged = false;
      } catch (const myactuator_rmd::can::Exception& e) {
        handle_error(e);
      } catch (const myactuator_rmd::Exception& e) {
        handle_error(e);
      } catch (const std::exception& e) {
        handle_error(e);
      }

      std::this_thread::sleep_until(wakeup_time);
    }
    actuator_interface_->setTimeout(std::chrono::milliseconds(0));
    return;
  }

  bool MyActuatorRmdHardwareInterface::startAsyncThread(std::chrono::milliseconds const& cycle_time) {
    if (!async_thread_.joinable()) {
      async_thread_ = std::thread(&MyActuatorRmdHardwareInterface::asyncThread, this, cycle_time);
    } else {
      RCLCPP_WARN(getLogger(), "Could not start command thread, command thread already running!");
      return false;
    }

#ifdef MYACTUATOR_RMD_HARDWARE__THREAD_PRIORITY
    std::ifstream realtime_file {"/sys/kernel/realtime", std::ios::in};
    bool has_realtime {false};
    if (realtime_file.is_open()) {
      realtime_file >> has_realtime;
    }

    int policy {};
    struct ::sched_param param {};
    ::pthread_getschedparam(async_thread_.native_handle(), &policy, &param);
    if (has_realtime) {
      policy = SCHED_FIFO;
      RCLCPP_INFO(getLogger(), "Real-time system detected: Setting policy to 'SCHED_FIFO'...");
    }
    int const max_thread_priority {::sched_get_priority_max(policy)};
    if (max_thread_priority != -1) {
      param.sched_priority = max_thread_priority;
      if (::pthread_setschedparam(async_thread_.native_handle(), policy, &param) == 0) {
        RCLCPP_INFO(getLogger(), "Set thread priority '%d' and policy '%d' to async thread!",
          param.sched_priority, policy);
      } else {
        RCLCPP_WARN(getLogger(), "Failed to set thread priority '%d' and policy '%d' to async thread!",
          param.sched_priority, policy);
      }
    } else {
      RCLCPP_WARN(getLogger(), "Could not set thread priority to async thread: Failed to get max priority!");
    }
#endif  // MYACTUATOR_RMD_HARDWARE__THREAD_PRIORITY

    return true;
  }

  void MyActuatorRmdHardwareInterface::stopAsyncThread() {
    if (async_thread_.joinable()) {
      stop_async_thread_.store(true);
      async_thread_.join();
    } else {
      RCLCPP_WARN(getLogger(), "Could not stop command thread: Not running!");
    }
    return;
  }

}  // namespace myactuator_rmd_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(myactuator_rmd_hardware::MyActuatorRmdHardwareInterface, hardware_interface::ActuatorInterface)
