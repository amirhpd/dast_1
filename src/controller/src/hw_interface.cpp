#include "controller/hw_interface.hpp"  // syntax error does not matter
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace hw_controller
{

HwInterface::HwInterface()
{
}

HwInterface::~HwInterface()
{
  if (nano33_.IsOpen())
  {
    try
    {
      nano33_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("HwInterface"),
                          "Failed to close port " << port_ << "while destructing HwInterface.");
    }
  }
}

CallbackReturn HwInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HwInterface"), "Serial port parameter not set.");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HwInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HwInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn HwInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "Activating hardware interface..");

  position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0 };

  try
  {
    nano33_.Open(port_);
    nano33_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HwInterface"),
                        "Failed to open port " << port_ << " while activating hardware interface.");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("HwInterface"),
              "Hardware interface activated successfully.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HwInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "Deactivating hardware interface..");

  if (nano33_.IsOpen())
  {
    try
    {
      nano33_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("HwInterface"),
                          "Failed to close port " << port_ << " while deactivating hardware interface.");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "Hardware deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HwInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  // Open loop control. No feedback from the hardware.
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HwInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  if (position_commands_ == prev_position_commands_)
  {
    return hardware_interface::return_type::OK;
  }

  int joint_1 = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  int joint_2 = static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  int joint_3 = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  int joint_4 = static_cast<int>(((position_commands_.at(3) + (M_PI / 2)) * 180) / M_PI);
  int joint_5 = static_cast<int>(((position_commands_.at(4) + (M_PI / 2)) * 180) / M_PI);

  std::string msg = 
    "J1" + std::to_string(joint_1) + "-" +
    "J2" + std::to_string(joint_2) + "-" +
    "J3" + std::to_string(joint_3) + "-" +
    "J4" + std::to_string(joint_4) + "-" +
    "J5" + std::to_string(joint_5);  

  try
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("HwInterface"), "Sending command: " << msg);
    nano33_.Write(msg);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HwInterface"), 
        "Failed to send command: " << msg << " to port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}

}  // namespace hw_controller
