#ifndef AIEAR_HARDWARE__AIEAR_SYSTEM_HPP_
#define AIEAR_HARDWARE__AIEAR_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace aiear_hardware
{

class AiearSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AiearSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool openSerial();
  void closeSerial();
  bool sendLine(const std::string & line);
  bool readLine(std::string & line, int timeout_ms);  // blocking; activation only
  void drainReplies();                                // non-blocking; RT loop

  // --- parameters (from <ros2_control> tag in the URDF) ---
  std::string device_;
  int baud_rate_{115200};
  double counts_per_rev_{618.0};
  int reply_timeout_ms_{25};

  int fd_{-1};  // serial file descriptor

  // Wheel order is fixed: FL, RL, FR, RR — matching the firmware reply.
  static constexpr int NUM_WHEELS = 4;
  static constexpr int FL = 0, RL = 1, FR = 2, RR = 3;

  std::array<std::string, NUM_WHEELS> joint_names_;
  std::array<double, NUM_WHEELS> pos_{};         // rad      (state)
  std::array<double, NUM_WHEELS> vel_{};         // rad/s    (state)
  std::array<double, NUM_WHEELS> cmd_{};         // rad/s    (command)
  std::array<long, NUM_WHEELS> counts_{};        // latest counts from firmware
  std::array<long, NUM_WHEELS> prev_counts_{};
  bool have_counts_{false};
  bool first_read_{true};

  std::string rx_buffer_;     // partial-line accumulator for drainReplies()
  int missed_replies_{0};     // consecutive write() cycles with no parsed reply
};

}  // namespace aiear_hardware

#endif  // AIEAR_HARDWARE__AIEAR_SYSTEM_HPP_
