// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "epmc_hardware_interface/epmc_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

namespace epmc_hardware_interface
{

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &info)
  {
    if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS )
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    config_.serial_port = info_.hardware_parameters["serial_port"];
    config_.serial_baud_rate = info_.hardware_parameters["serial_baud_rate"];
    config_.serial_timeout_ms = info_.hardware_parameters["serial_timeout_ms"];
    config_.motor0_wheel_name = info_.hardware_parameters["motor0_wheel_name"];
    config_.motor1_wheel_name = info_.hardware_parameters["motor1_wheel_name"];
    config_.cmd_vel_timeout_ms = info_.hardware_parameters["cmd_vel_timeout_ms"];

    if (config_.motor0_wheel_name != "") {
      use_motor0_ = true;
      motor0_.setup(config_.motor0_wheel_name);
    }
    if (config_.motor1_wheel_name != "") {
      use_motor1_ = true;
      motor1_.setup(config_.motor1_wheel_name);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EPMC_HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    if(use_motor0_){
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_POSITION, &motor0_.angPos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.angVel));
    }
    if(use_motor1_){
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_POSITION, &motor1_.angPos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.angVel));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EPMC_HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    if(use_motor0_){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.cmdAngVel));
    }
    if(use_motor1_){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.cmdAngVel));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Configuring ...please wait...");
    if (epmc_.connected())
    {
      epmc_.disconnect();
    }

    int serial_baud_rate = std::stoi(config_.serial_baud_rate.c_str());

    int serial_timeout_ms = std::stoi(config_.serial_timeout_ms.c_str());
    if (serial_timeout_ms <= 8)
      serial_timeout_ms = 8;

    epmc_.connect(config_.serial_port, serial_baud_rate, serial_timeout_ms);

    for (int i = 1; i <= 4; i += 1)
    { // wait for the smc to fully setup
      delay_ms(1000);
      RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "waiting for EPMC controller: %d sec", (i));
    }

    success = epmc_.clearDataBuffer();
    epmc_.writeSpeed(0.0, 0.0);

    int cmd_timeout = std::stoi(config_.cmd_vel_timeout_ms.c_str());
    epmc_.setCmdTimeout(cmd_timeout); // set motor command timeout
    std::tie(success, val0) = epmc_.getCmdTimeout();
    if (success) {
      cmd_timeout = val0;
      RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "motor_cmd_timeout_ms: %d ms", (cmd_timeout));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "ERROR: could not read motor_cmd_timeout_ms");
    }

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "EPMC Started Successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Cleaning up ...please wait...");
    if (epmc_.connected())
    {
      epmc_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Activating ...please wait...");
    if (!epmc_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    success = epmc_.clearDataBuffer();
    epmc_.writeSpeed(0.0, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Deactivating ...please wait...");

    success = epmc_.clearDataBuffer();
    epmc_.writeSpeed(0.0, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Successfully Deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EPMC_HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    std::tie(success, pos0_cache_, pos1_cache_, vel0_cache_, vel1_cache_) = epmc_.readMotorData();
    if (success) { // only update if read was successfull
      if(use_motor0_){
        motor0_.angPos = pos0_cache_;
        motor0_.angVel = vel0_cache_;
      }
      if(use_motor1_){
        motor1_.angPos = pos1_cache_;
        motor1_.angVel = vel1_cache_;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type epmc_hardware_interface ::EPMC_HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    if(use_motor0_){
      cmd0_cache_ = motor0_.cmdAngVel;
    }
    if(use_motor1_){
      cmd1_cache_ = motor1_.cmdAngVel;
    }
    epmc_.writeSpeed(cmd0_cache_, cmd1_cache_);

    return hardware_interface::return_type::OK;
  }

} // namespace epmc_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epmc_hardware_interface::EPMC_HardwareInterface, hardware_interface::SystemInterface)