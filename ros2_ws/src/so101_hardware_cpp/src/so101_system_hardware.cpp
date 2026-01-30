#include "so101_hardware_cpp/so101_system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "SMS_STS.h"
#include <fstream>
#include <cmath>
#include <nlohmann/json.hpp>

namespace so101_hardware_cpp
{

hardware_interface::CallbackReturn SO101SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  port_ = info_.hardware_parameters["port"];
  calib_file_ = info_.hardware_parameters["calib_file"];

  // Read optional reset_positions parameter (JSON string)
  reset_positions_str_ = "";
  if (info_.hardware_parameters.find("reset_positions") != info_.hardware_parameters.end())
  {
    reset_positions_str_ = info_.hardware_parameters["reset_positions"];
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  motor_ids_.resize(info_.joints.size());
  reset_positions_.resize(info_.joints.size(), 0.0);
  has_reset_positions_ = false;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    motor_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Configuring...");

  // Load calibration
  std::ifstream f(calib_file_);
  if (!f.is_open())
  {
    RCLCPP_WARN(rclcpp::get_logger("SO101SystemHardware"), "Calibration file not found: %s", calib_file_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto calib = nlohmann::json::parse(f);
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    std::string id_str = std::to_string(motor_ids_[i]);
    homing_offsets_[motor_ids_[i]] = calib[id_str]["homing_offset"];
    range_mins_[motor_ids_[i]] = calib[id_str]["range_min"];
    range_maxes_[motor_ids_[i]] = calib[id_str]["range_max"];
  }

  // Parse reset_positions if provided
  if (!reset_positions_str_.empty())
  {
    try
    {
      auto reset_json = nlohmann::json::parse(reset_positions_str_);
      // Reset positions can be formatted as "1": value, "2": value, etc.
      // Map joint IDs to array indices
      for (size_t i = 0; i < motor_ids_.size(); i++)
      {
        std::string id_str = std::to_string(motor_ids_[i]);
        if (reset_json.contains(id_str))
        {
          reset_positions_[i] = reset_json[id_str];
        }
      }
      has_reset_positions_ = true;
      RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Reset positions configured from parameter");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(rclcpp::get_logger("SO101SystemHardware"), "Failed to parse reset_positions JSON: %s", e.what());
      RCLCPP_WARN(rclcpp::get_logger("SO101SystemHardware"), "Will preserve current motor positions on startup");
      has_reset_positions_ = false;
    }
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "No reset_positions configured - will preserve current motor positions on startup");
    has_reset_positions_ = false;
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SO101SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Activating...");
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Port: %s", port_.c_str());

  if (!sms_sts_.begin(1000000, port_.c_str()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"), "Failed to connect to motors on port %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (has_reset_positions_)
  {
    // Move to configured reset positions
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Moving to configured reset positions...");
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
      // Convert radians to raw motor position
      double range = range_maxes_[motor_ids_[i]] - range_mins_[motor_ids_[i]];
      s16 pos = (reset_positions_[i] / (2.0 * M_PI) + 0.5) * range + range_mins_[motor_ids_[i]];
      sms_sts_.WritePosEx(motor_ids_[i], pos, 0, 0);

      // Initialize commands and positions to reset values
      hw_commands_[i] = reset_positions_[i];
      hw_positions_[i] = reset_positions_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Activated! Moving to reset positions.");
  }
  else
  {
    // Preserve current motor positions
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Reading current motor positions...");
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
      s16 pos = sms_sts_.ReadPos(motor_ids_[i]);
      if (pos != -1)
      {
        // Convert raw position to radians and set as initial command
        double range = range_maxes_[motor_ids_[i]] - range_mins_[motor_ids_[i]];
        double rad = ((pos - range_mins_[motor_ids_[i]]) / range - 0.5) * 2.0 * M_PI;
        hw_commands_[i] = rad;
        hw_positions_[i] = rad;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Activated! Preserving current motor positions.");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Deactivating...");

  // Disable torque on all motors before disconnecting
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    sms_sts_.EnableTorque(motor_ids_[i], 0);
  }

  // Wait a brief moment for torque to be disabled
  usleep(100000); // 100ms

  // Disconnect from serial port
  sms_sts_.end();

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Deactivated! Motors torques disabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SO101SystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    s16 pos = sms_sts_.ReadPos(motor_ids_[i]);
    if (pos != -1)
    {
      // Convert raw position to radians
      double range = range_maxes_[motor_ids_[i]] - range_mins_[motor_ids_[i]];
      hw_positions_[i] = ((pos - range_mins_[motor_ids_[i]]) / range - 0.5) * 2.0 * M_PI;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SO101SystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    // Convert radians to raw position
    double range = range_maxes_[motor_ids_[i]] - range_mins_[motor_ids_[i]];
    s16 pos = (hw_commands_[i] / (2.0 * M_PI) + 0.5) * range + range_mins_[motor_ids_[i]];
    sms_sts_.WritePosEx(motor_ids_[i], pos, 0, 0);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace so101_hardware_cpp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(so101_hardware_cpp::SO101SystemHardware, hardware_interface::SystemInterface)
