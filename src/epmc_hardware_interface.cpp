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
  auto epmcReadWriteTime = std::chrono::system_clock::now();
  std::chrono::duration<double> epmcReadWriteDuration;
  float epmcReadWriteTimeInterval = 0.015; // ~66Hz

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &info)
  {
    if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS )
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    config_.motor0_wheel_name = info_.hardware_parameters["motorA_wheel_name"];
    config_.motor1_wheel_name = info_.hardware_parameters["motorB_wheel_name"];
    config_.port = info_.hardware_parameters["port"];
    config_.cmd_vel_timeout_ms = info_.hardware_parameters["cmd_vel_timeout_ms"];

    motor0_.setup(config_.motor0_wheel_name);
    motor1_.setup(config_.motor1_wheel_name);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EPMC_HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_POSITION, &motor0_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.angVel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_POSITION, &motor1_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.angVel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EPMC_HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.cmdAngVel));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.cmdAngVel));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Configuring ...please wait...");
    if (epmc_.connected())
    {
      epmc_.disconnect();
    }
    epmc_.connect(config_.port);

    for (int i = 1; i <= 4; i += 1)
    { // wait for the smc to fully setup
      delay_ms(1000);
      RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "configuring controller: %d sec", (i));
    }

    epmc_.clearDataBuffer();
    epmc_.writeSpeed(0.0, 0.0);

    int cmd_timeout = std::stoi(config_.cmd_vel_timeout_ms.c_str());
    epmc_.setCmdTimeout(cmd_timeout); // set motor command timeout
    cmd_timeout = epmc_.getCmdTimeout();

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "motor_cmd_timeout_ms: %d ms", (cmd_timeout));

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Successfully configured!");

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

    epmc_.clearDataBuffer();
    epmc_.writeSpeed(0.0, 0.0);

    running_ = true;
    io_thread_ = std::thread(&EPMC_HardwareInterface::serialReadWriteLoop, this);
    epmcReadWriteTime = std::chrono::system_clock::now();

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Deactivating ...please wait...");

    running_ = false;
    if (io_thread_.joinable()) {
      io_thread_.join();
    }

    epmc_.clearDataBuffer();
    epmc_.writeSpeed(0.0, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("EPMC_HardwareInterface"), "Successfully Deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EPMC_HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

      motor0_.angPos = pos_cache_[0];
      motor0_.angVel = vel_cache_[0];

      motor1_.angPos = pos_cache_[1];
      motor1_.angVel = vel_cache_[1];

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type epmc_hardware_interface ::EPMC_HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    cmd_cache_[0] = motor0_.cmdAngVel;
    cmd_cache_[1] = motor1_.cmdAngVel;

    return hardware_interface::return_type::OK;
  }

  void EPMC_HardwareInterface::serialReadWriteLoop()
  {
    while (running_) {
      epmcReadWriteDuration = (std::chrono::system_clock::now() - epmcReadWriteTime);
      if (epmcReadWriteDuration.count() > epmcReadWriteTimeInterval)
      {
        try {
          float pos0, pos1, v0, v1;
          // Read latest state from hardware
          epmc_.readMotorData(pos0, pos1, v0, v1);
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pos_cache_[0] = pos0; pos_cache_[1] = pos1;
            vel_cache_[0] = v0;   vel_cache_[1] = v1;

            // Write latest commands
            epmc_.writeSpeed(cmd_cache_[0], cmd_cache_[1]);
          }
        }
        catch (...) {
          // Ignore read/write errors
        }
        epmcReadWriteTime = std::chrono::system_clock::now();
      }
    }
  }

} // namespace epmc_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epmc_hardware_interface::EPMC_HardwareInterface, hardware_interface::SystemInterface)
