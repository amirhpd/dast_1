#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>

namespace hw_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HwInterface : public hardware_interface::SystemInterface
{
public:
  HwInterface();
  virtual ~HwInterface();

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  virtual CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort nano33_;
  std::string port_;
  std::vector<std::string> position_interfaces_;  // "<joint>/position", one per joint
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<std::string> _split_string(const std::string& str, char delimiter);
};

} // namespace hw_controller
#endif  // HW_INTERFACE_H
