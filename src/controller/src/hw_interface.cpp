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

CallbackReturn HwInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
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

  // The framework exports the interfaces declared in <ros2_control>; we only keep their names
  // so read()/write() can address them with set_state()/get_command().
  for (const auto &joint : info_.joints)
  {
    position_interfaces_.push_back(joint.name + "/" + hardware_interface::HW_IF_POSITION);
  }

  position_commands_.assign(info_.joints.size(), 0.0);
  prev_position_commands_.assign(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn HwInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "Activating hardware interface..");

  const std::vector<double> home_position = { 0.0, 0.0, 1.571, 1.484, 0.0 };
  if (home_position.size() != position_interfaces_.size())
  {
    RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),
                 "Hardware has %zu joints but %zu home positions are defined.",
                 position_interfaces_.size(), home_position.size());
    return CallbackReturn::FAILURE;
  }

  position_commands_ = home_position;
  prev_position_commands_ = home_position;
  for (size_t i = 0; i < position_interfaces_.size(); i++)
  {
    set_state(position_interfaces_[i], home_position[i]);
    set_command(position_interfaces_[i], home_position[i]);
  }

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
  // simulated closed-loop feedback
  std::string feedback;
  try
  {
    nano33_.Read(feedback, 0, 100);
  }
  catch(const std::exception &e)
  {
    if (std::string(e.what()) != "Read timeout")
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("HwInterface"), "Feedback error: " << e.what());
      return hardware_interface::return_type::ERROR;
    }
    // waiting for feedback
  }

  if (!feedback.empty() && feedback[0] == 'F')
  {
    std::vector<std::string> feedbacks = _split_string(feedback, ',');
    feedbacks[0].erase(0, 1);
    for (size_t i = 0; i < feedbacks.size() && i < position_interfaces_.size(); i++)
    {
      set_state(position_interfaces_[i], std::stod(feedbacks[i]) * M_PI / 180);
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("HwInterface"), "Feedback: " << feedback);
  }

  // open-loop feedback
  // for (size_t i = 0; i < position_interfaces_.size(); i++)
  //   set_state(position_interfaces_[i], position_commands_[i]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HwInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  for (size_t i = 0; i < position_interfaces_.size(); i++)
  {
    position_commands_[i] = get_command<double>(position_interfaces_[i]);
  }

  if (position_commands_ == prev_position_commands_)
  {
    return hardware_interface::return_type::OK;
  }

  // wire format: comma-separated integer degrees, one per joint, no terminator
  std::string msg;
  for (size_t i = 0; i < position_commands_.size(); i++)
  {
    if (i > 0)
    {
      msg += ",";
    }
    msg += std::to_string(static_cast<int>(position_commands_[i] * (180 / M_PI)));
  }

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

std::vector<std::string> HwInterface::_split_string(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = str.find(delimiter);

    while (end != std::string::npos) {
        tokens.push_back(str.substr(start, end - start));
        start = end + 1;  // Move past the delimiter
        end = str.find(delimiter, start);
    }
    tokens.push_back(str.substr(start)); // Add the last token

    return tokens;
}

}  // namespace hw_controller
PLUGINLIB_EXPORT_CLASS(hw_controller::HwInterface, hardware_interface::SystemInterface)
