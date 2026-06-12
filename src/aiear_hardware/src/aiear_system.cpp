#include "aiear_hardware/aiear_system.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"

namespace aiear_hardware
{

static constexpr double TWO_PI = 2.0 * M_PI;

// ============================ Lifecycle ================================

hardware_interface::CallbackReturn AiearSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto logger = rclcpp::get_logger("AiearSystem");

  // --- hardware parameters from the URDF <ros2_control> block ---
  try {
    device_ = info_.hardware_parameters.at("device");
    counts_per_rev_ = std::stod(info_.hardware_parameters.at("counts_per_rev"));
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(logger, "Missing required hardware parameter 'device' or 'counts_per_rev'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }
  if (info_.hardware_parameters.count("reply_timeout_ms")) {
    reply_timeout_ms_ = std::stoi(info_.hardware_parameters.at("reply_timeout_ms"));
  }

  // --- map the four wheel joints by explicit parameters ---
  // URDF must provide: fl_joint, rl_joint, fr_joint, rr_joint
  const char * keys[NUM_WHEELS] = {"fl_joint", "rl_joint", "fr_joint", "rr_joint"};
  for (int i = 0; i < NUM_WHEELS; ++i) {
    try {
      joint_names_[i] = info_.hardware_parameters.at(keys[i]);
    } catch (const std::out_of_range &) {
      RCLCPP_FATAL(logger, "Missing hardware parameter '%s'", keys[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // --- sanity-check the joints declared in the URDF ---
  if (info_.joints.size() != NUM_WHEELS) {
    RCLCPP_FATAL(
      logger, "Expected %d joints in <ros2_control>, got %zu",
      NUM_WHEELS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    bool known = false;
    for (const auto & name : joint_names_) {
      if (joint.name == name) {known = true;}
    }
    if (!known) {
      RCLCPP_FATAL(
        logger, "Joint '%s' in URDF does not match any of fl/rl/fr/rr_joint params",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger, "Joint '%s' must have exactly one 'velocity' command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    logger, "AIEAR hardware initialised: device=%s, baud=%d, CPR=%.0f",
    device_.c_str(), baud_rate_, counts_per_rev_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AiearSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = rclcpp::get_logger("AiearSystem");

  if (!openSerial()) {
    RCLCPP_FATAL(logger, "Failed to open serial device %s", device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The ESP32 reboots when the port opens (DTR reset). Give it a moment,
  // then flush whatever boot noise it printed.
  rclcpp::sleep_for(std::chrono::milliseconds(1500));
  tcflush(fd_, TCIOFLUSH);

  // Reset commands and confirm the firmware is talking.
  sendLine("s\n");
  std::string reply;
  if (!readLine(reply, 500)) {
    RCLCPP_FATAL(logger, "No response from firmware to stop command");
    closeSerial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & c : cmd_) {c = 0.0;}
  first_read_ = true;
  have_counts_ = false;

  RCLCPP_INFO(logger, "AIEAR hardware activated, firmware responding");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AiearSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0) {
    sendLine("s\n");
    closeSerial();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ======================= Interface export ==============================

std::vector<hardware_interface::StateInterface> AiearSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (int i = 0; i < NUM_WHEELS; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AiearSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (int i = 0; i < NUM_WHEELS; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_[i]);
  }
  return interfaces;
}

// ============================ read / write =============================
// Per controller-manager cycle: read() runs first (we convert the counts
// captured during the *previous* write()'s reply), then controllers
// update, then write() sends new commands and captures fresh counts.

hardware_interface::return_type AiearSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!have_counts_) {
    return hardware_interface::return_type::OK;  // nothing received yet
  }

  const double rad_per_count = TWO_PI / counts_per_rev_;
  const double dt = period.seconds();

  for (int i = 0; i < NUM_WHEELS; ++i) {
    pos_[i] = counts_[i] * rad_per_count;
    if (first_read_ || dt <= 0.0) {
      vel_[i] = 0.0;
    } else {
      vel_[i] = (counts_[i] - prev_counts_[i]) * rad_per_count / dt;
    }
    prev_counts_[i] = counts_[i];
  }
  first_read_ = false;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AiearSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 1) Consume any replies that arrived since the last cycle. Never block:
  //    a slow USB reply just means this cycle reuses last cycle's counts.
  drainReplies();

  // 2) Send this cycle's command. diff_drive_controller writes identical
  //    commands to both wheels of a side, so the first joint of each side
  //    represents the side command.
  const double counts_per_rad = counts_per_rev_ / TWO_PI;
  const double left_cps = cmd_[FL] * counts_per_rad;    // rad/s -> counts/s
  const double right_cps = cmd_[FR] * counts_per_rad;

  char buf[64];
  std::snprintf(buf, sizeof(buf), "v %.1f %.1f\n", left_cps, right_cps);
  if (!sendLine(buf)) {
    RCLCPP_WARN(rclcpp::get_logger("AiearSystem"), "Serial write failed");
    return hardware_interface::return_type::ERROR;
  }

  // 3) Health check without any clock objects in the RT thread: plain
  //    cycle counting. ~90 cycles at 30 Hz is ~3 s of silence.
  if (++missed_replies_ == 90) {
    RCLCPP_WARN(
      rclcpp::get_logger("AiearSystem"),
      "No encoder replies parsed for ~3 s — check ESP32 link");
    missed_replies_ = 0;
  }

  return hardware_interface::return_type::OK;
}

// Non-blocking: read whatever bytes are available, split into lines,
// parse every complete reply, keep the newest counts.
void AiearSystem::drainReplies()
{
  if (fd_ < 0) {return;}

  char chunk[256];
  ssize_t n;
  while ((n = ::read(fd_, chunk, sizeof(chunk))) > 0) {
    rx_buffer_.append(chunk, n);
  }

  size_t pos;
  while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
    std::string line = rx_buffer_.substr(0, pos);
    rx_buffer_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r') {line.pop_back();}

    long fl, rl, fr, rr;
    if (std::sscanf(line.c_str(), "%ld %ld %ld %ld", &fl, &rl, &fr, &rr) == 4) {
      counts_[FL] = fl;  counts_[RL] = rl;
      counts_[FR] = fr;  counts_[RR] = rr;
      have_counts_ = true;
      missed_replies_ = 0;
    }
    // Non-matching lines (e.g. an "ok") are simply dropped.
  }

  if (rx_buffer_.size() > 1024) {rx_buffer_.clear();}  // runaway garbage guard
}

// ============================ Serial I/O ===============================

bool AiearSystem::openSerial()
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {return false;}

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {closeSerial(); return false;}

  speed_t speed = B115200;
  switch (baud_rate_) {
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger("AiearSystem"),
        "Unsupported baud %d, falling back to 115200", baud_rate_);
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, raw mode, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;   // non-blocking reads; timeouts handled via select()
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {closeSerial(); return false;}
  return true;
}

void AiearSystem::closeSerial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool AiearSystem::sendLine(const std::string & line)
{
  if (fd_ < 0) {return false;}
  ssize_t n = ::write(fd_, line.c_str(), line.size());
  return n == static_cast<ssize_t>(line.size());
}

bool AiearSystem::readLine(std::string & line, int timeout_ms)
{
  line.clear();
  if (fd_ < 0) {return false;}

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - std::chrono::steady_clock::now());
    timeval tv;
    tv.tv_sec = remaining.count() / 1000000;
    tv.tv_usec = remaining.count() % 1000000;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    if (select(fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
      return false;  // timeout or error
    }

    char c;
    while (::read(fd_, &c, 1) == 1) {
      if (c == '\n') {
        return !line.empty();
      }
      if (c != '\r') {
        line.push_back(c);
      }
      if (line.size() > 128) {line.clear();}  // runaway garbage guard
    }
  }
  return false;
}

}  // namespace aiear_hardware

PLUGINLIB_EXPORT_CLASS(aiear_hardware::AiearSystem, hardware_interface::SystemInterface)
