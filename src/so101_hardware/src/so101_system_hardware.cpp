#include "so101_hardware/so101_system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "SMS_STS.h"
#include <fstream>
#include <cmath>
#include <nlohmann/json.hpp>

namespace so101_hardware
{

hardware_interface::CallbackReturn SO101SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  port_ = info_.hardware_parameters["port"];
  calib_file_ = info_.hardware_parameters["calib_file"];
  reset_positions_str_ = info_.hardware_parameters["reset_positions"];

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  motor_ids_.resize(info_.joints.size());
  target_positions_.resize(info_.joints.size(), 0);
  target_speeds_.resize(info_.joints.size(), 0);
  target_accs_.resize(info_.joints.size(), 0);
  reset_positions_.resize(info_.joints.size(), 0.0);
  has_reset_positions_ = false;

  try {
    if (!reset_positions_str_.empty() && reset_positions_str_ != "''" && reset_positions_str_ != "\"\"") {
      auto reset_json = nlohmann::json::parse(reset_positions_str_);
      for (size_t i = 0; i < info_.joints.size(); i++) {
        std::string joint_name = info_.joints[i].name;
        if (reset_json.contains(joint_name)) {
          reset_positions_[i] = reset_json[joint_name];
          has_reset_positions_ = true;
        }
      }
      if (has_reset_positions_) {
        RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Loaded reset positions from config");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("SO101SystemHardware"), "Failed to parse reset_positions: %s", e.what());
  }

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

  if (!sms_sts_.begin(1000000, port_.c_str()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"), "Failed to connect to motors on port %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 0. Robustness check: Ping each motor to ensure it is connected and responsive
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    u8 id = motor_ids_[i];
    int retry = 3;
    bool found = false;
    while (retry--) {
      if (sms_sts_.Ping(id) != -1) {
        found = true;
        break;
      }
      usleep(10000); // 10ms wait between retries
    }
    
    if (!found) {
      RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"), 
        "Motor ID %d is NOT responding! Check cables and power.", id);
      return hardware_interface::CallbackReturn::FAILURE;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("SO101SystemHardware"), "Motor ID %d found.", id);
  }

  const double TICKS_PER_RAD = 4096.0 / (2.0 * M_PI);

  // 1. Configure Hardware: Write Offsets, PID, and Return Delay
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    u8 id = motor_ids_[i];
    
    // 1.1 Disable torque before configuration
    sms_sts_.EnableTorque(id, 0); 
    usleep(2000); // Small delay
    
    // 1.2 Unlock EPROM to allow parameter writing
    sms_sts_.unLockEprom(id);
    usleep(2000);
    
    // CORRECT Sign-Magnitude encoding for STS series (Sign bit is 11)
    int offset = homing_offsets_[id];
    u16 encoded_offset = (offset < 0) ? (static_cast<u16>(std::abs(offset)) | (1 << 11)) : static_cast<u16>(offset);
    
    RCLCPP_DEBUG(rclcpp::get_logger("SO101SystemHardware"), "Setting ID %d: Homing Offset=%d (Encoded: %u)", id, offset, encoded_offset);
    
    sms_sts_.writeWord(id, 31, encoded_offset); 
    sms_sts_.writeWord(id, 9, range_mins_[id]);   
    sms_sts_.writeWord(id, 11, range_maxes_[id]); 
    
    sms_sts_.writeByte(id, 7, 0);   
    sms_sts_.writeByte(id, 21, 16); 
    sms_sts_.writeByte(id, 22, 32); 
    sms_sts_.writeByte(id, 23, 0);  
    usleep(2000);

    // 1.3 Lock EPROM after configuration to persist parameters
    sms_sts_.LockEprom(id);
    usleep(2000);
    
    // 1.4 Enable torque after configuration
    sms_sts_.EnableTorque(id, 1);
    usleep(2000);
  }

  // 2. Initialize sync read buffer (Reduced timeout to 10ms for stability)
  sms_sts_.syncReadBegin(motor_ids_.size(), 2, 10);

  // Initial sync
  if (sms_sts_.syncReadPacketTx(motor_ids_.data(), motor_ids_.size(), 56, 2) > 0)
  {
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
      u8 id = motor_ids_[i];
      u8 data[2];
      if (sms_sts_.syncReadPacketRx(id, data) == 2)
      {
        s16 pos = (data[1] << 8) | data[0];
        double rad = (static_cast<double>(pos) - 2048.0) / TICKS_PER_RAD;
        
        // Use reset position if specified, otherwise stay at current position
        if (has_reset_positions_) {
          hw_commands_[i] = reset_positions_[i];
          RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Initial command set to RESET position: %.4f (Current RAW: %d)", reset_positions_[i], pos);
        } else {
          hw_commands_[i] = rad;
          RCLCPP_DEBUG(rclcpp::get_logger("SO101SystemHardware"), "Initial Sync ID %d: RAW=%d -> RAD=%.4f", id, pos, rad);
        }
        hw_positions_[i] = rad;
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Activated! Control loop running.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Deactivating...");
  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    sms_sts_.EnableTorque(motor_ids_[i], 0);
  }
  usleep(100000);
  sms_sts_.syncReadEnd();
  sms_sts_.end();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SO101SystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  int read_len = sms_sts_.syncReadPacketTx(motor_ids_.data(), motor_ids_.size(), 56, 2);
  if (read_len <= 0) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SO101SystemHardware"), steady_clock, 500, "SyncRead PacketTx FAILED");
    return hardware_interface::return_type::OK;
  }

  const double TICKS_PER_RAD = 4096.0 / (2.0 * M_PI);

  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    u8 data[2];
    if (sms_sts_.syncReadPacketRx(motor_ids_[i], data) == 2)
    {
      s16 pos = (data[1] << 8) | data[0];
      hw_positions_[i] = (static_cast<double>(pos) - 2048.0) / TICKS_PER_RAD;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SO101SystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  const double TICKS_PER_RAD = 4096.0 / (2.0 * M_PI);

  for (size_t i = 0; i < motor_ids_.size(); i++)
  {
    double target_raw = hw_commands_[i] * TICKS_PER_RAD + 2048.0;
    
    // Safety clamp to [0, 4095]
    if (target_raw < 0) target_raw = 0;
    if (target_raw > 4095) target_raw = 4095;

    target_positions_[i] = static_cast<s16>(target_raw);
    target_speeds_[i] = 2400; 
    target_accs_[i] = 50;
  }

  sms_sts_.SyncWritePosEx(motor_ids_.data(), motor_ids_.size(), target_positions_.data(), target_speeds_.data(), target_accs_.data());
  
  return hardware_interface::return_type::OK;
}

}  // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101SystemHardware, hardware_interface::SystemInterface)