/**
 * \file myactuator_rmd_hardware_interface.hpp
 * \mainpage
 *    ROS 2 Control hardware interface for MyActuator RMD-X series actuators based on CAN
 *    See https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
#define MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/driver/can_driver.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "myactuator_rmd_hardware/low_pass_filter.hpp"
#include "myactuator_rmd_hardware/velocity_estimators.hpp"
#include "myactuator_rmd_hardware/visibility_control.hpp"


namespace myactuator_rmd_hardware {

  /**\class MyActuatorRmdHardwareInterface
   * \brief
   *    Hardware interface for the MyActuator RMD X-series actuators based on CAN
  */
  class MyActuatorRmdHardwareInterface: public hardware_interface::ActuatorInterface {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(MyActuatorRmdHardwareInterface)
      
      MyActuatorRmdHardwareInterface() = default;
      MyActuatorRmdHardwareInterface(MyActuatorRmdHardwareInterface const&) = default;
      MyActuatorRmdHardwareInterface& operator = (MyActuatorRmdHardwareInterface const&) = default;
      MyActuatorRmdHardwareInterface(MyActuatorRmdHardwareInterface&&) = default;
      MyActuatorRmdHardwareInterface& operator = (MyActuatorRmdHardwareInterface&&) = default;

      /**\fn ~MyActuatorRmdHardwareInterface
       * \brief
       *    Class destructor
       *    Makes sure the actuator is shut down cleanly
      */
      ~MyActuatorRmdHardwareInterface();

      /**\fn on_configure
       * \brief
       *    Callback function for configure transition
       *    Sets up communication with the hardware
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_configure(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_cleanup
       * \brief
       *    Callback function for cleanup transition
       *    Gracefully shuts down the actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_cleanup(rclcpp_lifecycle::State const& previous_state) override;
      
      /**\fn on_shutdown
       * \brief
       *    Callback function for shutdown transition
       *    Gracefully shuts down the actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_shutdown(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_activate
       * \brief
       *    Callback function for activate transition
       *    Enables hardware power
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_activate(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_deactivate
       * \brief
       *    Callback function for deactivate transition
       *    Disables hardware power
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_deactivate(rclcpp_lifecycle::State const& previous_state) override;
      
      /**\fn on_error
       * \brief
       *    Callback function for error transition
       *    Resets actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_error(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_init
       * \brief
       *    Callback function for init transition
       *    Initializes all member variables
       * 
       * \param[in] info
       *    Hardware info containing the information from the URDF
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      CallbackReturn on_init(hardware_interface::HardwareInfo const& info) override;

      /**\fn export_state_interfaces
       * \brief
       *    Export the state interfaces that the actuator exposes
       * 
       * \return
       *    The state interfaces the actuator exposes
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      /**\fn export_command_interfaces
       * \brief
       *    Export the command interfaces that the actuator exposes
       * 
       * \return
       *    The command interfaces the actuator exposes
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      /**\fn prepare_command_mode_switch
       * \brief
       *    Prepare the switch of the command mode of the actuator
       * 
       * \param[in] start_interfaces
       *    Interfaces to be started
       * \param[in] stop_interfaces
       *    Interfaces to be stopped
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      hardware_interface::return_type prepare_command_mode_switch(std::vector<std::string> const& start_interfaces,
        std::vector<std::string> const& stop_interfaces) override;

      /**\fn perform_command_mode_switch
       * \brief
       *    Switch of the command mode of the actuator
       * 
       * \param[in] start_interfaces
       *    Interfaces to be started
       * \param[in] stop_interfaces
       *    Interfaces to be stopped
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      hardware_interface::return_type perform_command_mode_switch(std::vector<std::string> const& start_interfaces,
        std::vector<std::string> const& stop_interfaces) override;

      /**\fn read
       * \brief
       *    Read the current state of the actuator
       * 
       * \param[in] time
       *    The current time
       * \param[in] period
       *    The time passed since the last read
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      hardware_interface::return_type read(rclcpp::Time const& time, rclcpp::Duration const& period) override;

      /**\fn write
       * \brief
       *    Write the next command to the actuator
       * 
       * \param[in] time
       *    The current time
       * \param[in] period
       *    The time passed since the last write
       * \return
       *    Value indicating success, error or failure of the callback
      */
      MYACTUATOR_RMD_HARDWARE_PUBLIC
      hardware_interface::return_type write(rclcpp::Time const& time, rclcpp::Duration const& period) override;
  
    protected:      
      /**\fn getLogger
       * \brief
       *    Get the logger used for console output
       * 
       * \return
       *    The logger
      */
      static rclcpp::Logger getLogger();

      /**\fn asyncThread
       * \brief
       *    The asynchronous command thread used to communicate with the hardware
       *    that performs a combined read and write
       * 
       * \param[in] cycle_time
       *    The cycle time that the asynchronous thread should run at
      */
      void asyncThread(std::chrono::milliseconds const& cycle_time);

      /**\fn startAsyncThread
       * \brief
       *    Start the asynchronous command thread used to communicate with the hardware
       * 
       * \param[in] cycle_time
       *    The cycle time that the asynchronous thread should run at
       * \return
       *    Boolean variable indicating successful start of the async thread or failure
      */
      [[nodiscard]]
      bool startAsyncThread(std::chrono::milliseconds const& cycle_time);
      
      /**\fn stopAsyncThread
       * \brief
       *    Stop the asynchronous command thread used to communicate with the hardware
      */
      void stopAsyncThread();
      
      std::string ifname_;
      std::uint32_t actuator_id_;
      // Hardware-level mounting sign: maps between the motor frame and the
      // canonical ROS-side convention. read: joint_state = direction_sign_ *
      // raw; write: motor_cmd = direction_sign_ * canonical. +-1.0 only.
      double direction_sign_{1.0};
      double torque_constant_;
      double max_current_;  // effort-branch current clamp [A] (kt-independent guard, ADR 0008)
      double max_velocity_;
      std::chrono::milliseconds timeout_;

      // Buffers only used by the main thread
      double position_state_;
      double velocity_state_;
      double effort_state_;
      double position_command_;
      double velocity_command_;
      double effort_command_;
      // Motion mode commands (position [rad], velocity [rad/s], kp [raw 0..4095], kd [raw 0..4095], torque_ff [Nm])
      double motion_p_command_;
      double motion_v_command_;
      double motion_kp_command_;
      double motion_kd_command_;
      double motion_tff_command_;
      std::unique_ptr<LowPassFilter> velocity_low_pass_filter_;
      std::unique_ptr<LowPassFilter> effort_low_pass_filter_;

      // The command thread reads and writes from the actuator cyclically
      std::thread async_thread_;
      std::chrono::milliseconds cycle_time_;
      // Never accessed by both threads at the same time
      std::unique_ptr<myactuator_rmd::CanDriver> driver_;
      std::unique_ptr<myactuator_rmd::ActuatorInterface> actuator_interface_;
      // Motion mode actuator interface
      std::unique_ptr<myactuator_rmd::motion_mode::CanDriver> motion_driver_;
      std::unique_ptr<myactuator_rmd::motion_mode::ActuatorInterface> motion_actuator_interface_;
      myactuator_rmd::Feedback feedback_;
      // Shared between the two threads
      std::atomic<bool> stop_async_thread_;
      
      std::atomic<double> async_position_state_;
      std::atomic<double> async_velocity_state_;
      std::atomic<double> async_effort_state_;
      std::atomic<double> async_position_command_;
      std::atomic<double> async_velocity_command_;
      std::atomic<double> async_effort_command_;
      std::atomic<double> async_motion_p_command_;
      std::atomic<double> async_motion_v_command_;
      std::atomic<double> async_motion_kp_command_;
      std::atomic<double> async_motion_kd_command_;
      std::atomic<double> async_motion_tff_command_;
      std::atomic<bool> position_interface_running_;
      std::atomic<bool> velocity_interface_running_;
      std::atomic<bool> effort_interface_running_;
      std::atomic<bool> motion_interface_running_;

      std::atomic<bool> motor_online_{false};
      std::atomic<bool> motor_first_reading_valid_{false};
      // Latched true once the motor has produced at least one valid reading.
      // Lets read() tell "never online yet" (startup) apart from "was online, now
      // dropped out" so it can export NaN state on drop-out and drive the core to
      // FAULT rather than freezing the last finite value (ADR 0008 mitigation).
      std::atomic<bool> ever_valid_reading_{false};

      // 1-Euro velocity estimator (encoder-based, used in motion mode)
      rclcpp::Node::SharedPtr param_node_;
      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
      std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> param_executor_;
      std::thread param_spin_thread_;
      std::unique_ptr<OneEuroEstimator> velocity_estimator_;

      // Debug publisher for raw motion mode data
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motion_debug_pub_;

      // Raw multi-turn encoder position as received from the driver each async
      // read, stamped at receive time. Feeds offline tuning of the velocity
      // estimate filter (the OneEuroEstimator assumes a fixed dt; this carries
      // the real per-sample timing the broadcaster-decimated /joint_states loses).
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_encoder_pub_;

  };

}  // namespace myactuator_rmd_hardware

#endif  // MYACTUATOR_RMD_HARDWARE__MYACTUATOR_RMD_HARDWARE_INTERFACE
