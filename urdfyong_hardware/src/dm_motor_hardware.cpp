// Copyright 2026 hibikip
//
// DmMotorHardware — ros2_control SystemInterface for DM-series motors
// via USB2CANFD (u2can serial CDC ACM protocol at 921600 baud).
//
// Licensed under the Apache License, Version 2.0

#include "urdfyong_hardware/dm_motor_hardware.hpp"

#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>
#include <stdexcept>
#include <cmath>

// Serial port header
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace urdfyong_hardware
{

// ── Static members ──────────────────────────────────────────────────
DmMotorHardware * DmMotorHardware::instance_ = nullptr;

std::shared_ptr<rclcpp::Node> DmMotorHardware::g_cmd_node_ = nullptr;
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>> DmMotorHardware::g_cmd_sub_ = nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> DmMotorHardware::g_cmd_executor_ = nullptr;
std::thread DmMotorHardware::g_cmd_thread_;

// ── External command bridge ─────────────────────────────────────────

void DmMotorHardware::start_cmd_bridge()
{
  if (g_cmd_node_) return;

  g_cmd_node_ = std::make_shared<rclcpp::Node>("dm_hardware_cmd_bridge");
  g_cmd_sub_ = g_cmd_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/position_commands", rclcpp::SystemDefaultsQoS(),
    [](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      auto * hw = instance_;
      if (!hw || !hw->motors_enabled_) return;
      std::lock_guard<std::mutex> lock(hw->feedback_mutex_);
      size_t n = std::min(msg->data.size(), hw->hw_position_commands_.size());
      for (size_t i = 0; i < n; ++i) {
        hw->hw_position_commands_[i] = msg->data[i];
      }
    });

  g_cmd_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  g_cmd_executor_->add_node(g_cmd_node_);
  g_cmd_thread_ = std::thread([]() { g_cmd_executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger("DmMotorHardware"),
    "Command bridge started — listening on /position_commands");
}

void DmMotorHardware::stop_cmd_bridge()
{
  if (!g_cmd_node_) return;
  g_cmd_executor_->cancel();
  if (g_cmd_thread_.joinable()) g_cmd_thread_.join();
  g_cmd_sub_.reset();
  g_cmd_node_.reset();
  g_cmd_executor_.reset();
  RCLCPP_INFO(rclcpp::get_logger("DmMotorHardware"), "Command bridge stopped");
}

// ── Constants ──────────────────────────────────────────────────────
static constexpr int NUM_JOINTS = 6;

// ── Destructor ─────────────────────────────────────────────────────
DmMotorHardware::~DmMotorHardware()
{
  if (instance_ == this) instance_ = nullptr;
}

// ═══════════════════════════════════════════════════════════════════
//  Serial port helpers
// ═══════════════════════════════════════════════════════════════════

bool DmMotorHardware::serial_open(const std::string &port, int baudrate)
{
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(logger_, "serial_open: cannot open %s (errno=%d: %s)",
                 port.c_str(), errno, strerror(errno));
    return false;
  }

  struct termios opt;
  std::memset(&opt, 0, sizeof(opt));
  tcgetattr(serial_fd_, &opt);

  cfsetispeed(&opt, baudrate);
  cfsetospeed(&opt, baudrate);

  opt.c_cflag &= ~CSIZE;
  opt.c_cflag |= CS8;          // 8 data bits
  opt.c_cflag &= ~PARENB;      // no parity
  opt.c_iflag &= ~INPCK;       // no parity checking
  opt.c_cflag &= ~CSTOPB;      // 1 stop bit
  opt.c_oflag = 0;
  opt.c_lflag = 0;
  opt.c_iflag = 0;

  opt.c_cc[VTIME] = 0;
  opt.c_cc[VMIN]  = 0;
  opt.c_lflag |= CBAUDEX;

  tcflush(serial_fd_, TCIFLUSH);
  tcsetattr(serial_fd_, TCSANOW, &opt);

  RCLCPP_INFO(logger_, "serial_open: %s @ %d baud, fd=%d",
              port.c_str(), baudrate, serial_fd_);
  return true;
}

void DmMotorHardware::serial_close()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(logger_, "serial_close: port closed");
  }
}

bool DmMotorHardware::serial_send(const U2CanTxFrame &frame)
{
  if (serial_fd_ < 0) return false;

  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t ret = ::write(serial_fd_, &frame, sizeof(U2CanTxFrame));
  if (ret != sizeof(U2CanTxFrame)) {
    RCLCPP_WARN(logger_, "serial_send: wrote %zd/%zu bytes", ret, sizeof(U2CanTxFrame));
    return false;
  }
  return true;
}

bool DmMotorHardware::serial_recv(U2CanRxFrame &frame, int timeout_ms)
{
  if (serial_fd_ < 0) return false;

  // Wait for data with select()
  fd_set rset;
  FD_ZERO(&rset);
  FD_SET(serial_fd_, &rset);
  timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int sel_ret = select(serial_fd_ + 1, &rset, nullptr, nullptr,
                       (timeout_ms >= 0) ? &tv : nullptr);
  if (sel_ret <= 0) return false;  // timeout or error

  // Read and buffer all available bytes
  uint8_t buf[256];
  ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
  for (ssize_t i = 0; i < n; ++i) {
    rx_queue_.push(buf[i]);
  }

  // Find a complete frame: 0xAA ... 0x55 (16 bytes)
  constexpr int FRAME_SIZE = sizeof(U2CanRxFrame);

  while (rx_queue_.size() >= static_cast<size_t>(FRAME_SIZE))
  {
    // Find 0xAA header
    while (!rx_queue_.empty() && rx_queue_.front() != 0xAA) {
      rx_queue_.pop();
    }
    if (rx_queue_.size() < static_cast<size_t>(FRAME_SIZE)) break;

    // Copy FRAME_SIZE bytes to check
    std::vector<uint8_t> peek(FRAME_SIZE);
    for (int i = 0; i < FRAME_SIZE; ++i) {
      peek[i] = rx_queue_.front();
      rx_queue_.push(rx_queue_.front());  // rotate: pop+push to back
      rx_queue_.pop();
    }

    // After rotate, queue is in original order
    if (peek[0] == 0xAA && peek[FRAME_SIZE - 1] == 0x55) {
      // Valid frame — pop from queue and return
      for (int i = 0; i < FRAME_SIZE; ++i) {
        reinterpret_cast<uint8_t*>(&frame)[i] = rx_queue_.front();
        rx_queue_.pop();
      }
      return true;
    }
    // Footer mismatch — pop the 0xAA header and retry
    rx_queue_.pop();
  }

  return false;
}

// ═══════════════════════════════════════════════════════════════════
//  CAN send helpers
// ═══════════════════════════════════════════════════════════════════

void DmMotorHardware::send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t dlen)
{
  U2CanTxFrame tx;
  tx.set_can_frame(can_id, data, dlen);
  serial_send(tx);
}

void DmMotorHardware::send_enable(uint8_t motor_index)
{
  if (motor_index >= motor_params_.size()) return;
  const uint32_t id = motor_params_[motor_index].can_id;
  send_can_frame(id, CMD_ENABLE, 8);
}

void DmMotorHardware::send_disable(uint8_t motor_index)
{
  if (motor_index >= motor_params_.size()) return;
  const uint32_t id = motor_params_[motor_index].can_id;
  send_can_frame(id, CMD_DISABLE, 8);
}

void DmMotorHardware::send_position_cmd(uint8_t motor_index, float p_des, float v_des)
{
  if (motor_index >= motor_params_.size()) return;
  if (!device_open_) return;

  uint8_t data[8] = {};
  pack_float_le(&data[0], p_des);
  pack_float_le(&data[4], v_des);

  const uint32_t can_id = CAN_ID_POS + motor_params_[motor_index].can_id;
  send_can_frame(can_id, data, 8);
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════

hardware_interface::CallbackReturn DmMotorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger_, "on_init: base class on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "on_init: hardware name = '%s'", info_.name.c_str());

  // Parse optional global parameters
  auto it = info_.hardware_parameters.find("serial_port");
  if (it != info_.hardware_parameters.end()) serial_port_ = it->second;

  it = info_.hardware_parameters.find("serial_baud");
  if (it != info_.hardware_parameters.end()) serial_baud_ = std::stoi(it->second);

  RCLCPP_INFO(logger_, "on_init: serial=%s, baud=%d", serial_port_.c_str(), serial_baud_);

  // Parse per-joint parameters
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(logger_, "on_init: expected %d joints, got %zu",
                 NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  motor_params_.reserve(NUM_JOINTS);
  motor_feedback_.resize(NUM_JOINTS);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & j = info_.joints[i];
    MotorParams p;
    try {
      p.can_id  = static_cast<uint8_t>(std::stoi(j.parameters.at("motor_can_id")));
      p.pos_min = std::stof(j.parameters.at("position_min"));
      p.pos_max = std::stof(j.parameters.at("position_max"));
      p.vel_max = std::stof(j.parameters.at("velocity_max"));
      p.trq_max = std::stof(j.parameters.at("torque_max"));
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(logger_, "on_init: joint '%s' missing param '%s'",
                   j.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Optional: motor_direction (-1 reverses, +1 normal)
    auto dir_it = j.parameters.find("motor_direction");
    if (dir_it != j.parameters.end()) {
      p.motor_direction = std::stof(dir_it->second);
    }

    RCLCPP_INFO(logger_, "on_init: joint '%s' → CAN ID=0x%02X, "
                "pos=[%.2f, %.2f] rad, vel_max=%.1f rad/s, trq_max=%.1f Nm, dir=%+.0f",
                j.name.c_str(), p.can_id, p.pos_min, p.pos_max,
                p.vel_max, p.trq_max, static_cast<double>(p.motor_direction));
    motor_params_.push_back(p);
  }

  // Allocate state / command storage
  hw_position_states_.resize(NUM_JOINTS, 0.0);
  hw_velocity_states_.resize(NUM_JOINTS, 0.0);
  hw_position_commands_.resize(NUM_JOINTS, 0.0);
  motor_alive_.resize(NUM_JOINTS, false);

  RCLCPP_INFO(logger_, "on_init: SUCCESS");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmMotorHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_configure: opening serial port...");

  // Setup singleton + command bridge
  instance_ = this;
  start_cmd_bridge();

  // Open serial port
  if (!serial_open(serial_port_, serial_baud_)) {
    RCLCPP_WARN(logger_, "on_configure: serial open failed — DRY-RUN mode");
    device_open_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  device_open_ = true;

  RCLCPP_INFO(logger_, "on_configure: SUCCESS");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmMotorHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_activate: enabling all motors...");

  if (!device_open_) {
    RCLCPP_WARN(logger_, "on_activate: no device — DRY-RUN mode");
    motors_enabled_ = true;
    first_read_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  for (size_t i = 0; i < motor_params_.size(); ++i) {
    const uint8_t motor_id = motor_params_[i].can_id;

    // Clear errors
    send_can_frame(motor_id, CMD_CLEAR_ERROR, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Enable motor
    send_can_frame(motor_id, CMD_ENABLE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    RCLCPP_INFO(logger_, "on_activate: motor CAN ID=0x%02X enabled", motor_id);
  }

  motors_enabled_ = true;
  first_read_ = true;

  RCLCPP_INFO(logger_, "on_activate: SUCCESS — %zu motors enabled",
              motor_params_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmMotorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_deactivate: disabling all motors...");
  motors_enabled_ = false;

  if (device_open_) {
    for (size_t i = 0; i < motor_params_.size(); ++i) {
      send_can_frame(motor_params_[i].can_id, CMD_DISABLE, 8);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  RCLCPP_INFO(logger_, "on_deactivate: SUCCESS");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmMotorHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_cleanup: closing...");

  stop_cmd_bridge();
  instance_ = nullptr;
  serial_close();
  device_open_ = false;

  RCLCPP_INFO(logger_, "on_cleanup: SUCCESS");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmMotorHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (motors_enabled_ && device_open_) {
    for (size_t i = 0; i < motor_params_.size(); ++i) {
      send_can_frame(motor_params_[i].can_id, CMD_DISABLE, 8);
    }
    motors_enabled_ = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
//  Interface export
// ═══════════════════════════════════════════════════════════════════

std::vector<hardware_interface::StateInterface>
DmMotorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(NUM_JOINTS * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & joint_name = info_.joints[i].name;
    state_interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);
    state_interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DmMotorHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(NUM_JOINTS);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
  }
  return command_interfaces;
}

// ═══════════════════════════════════════════════════════════════════
//  read() — send position cmd, then receive feedback
// ═══════════════════════════════════════════════════════════════════

hardware_interface::return_type DmMotorHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (first_read_) {
    first_read_time_ = time;
    first_read_ = false;
  }

  // ── Send position commands (these also trigger feedback) ──────
  // Send one command at a time and wait briefly for the response.
  // The motor is query-response: each command triggers one feedback frame.
  if (motors_enabled_ && device_open_) {
    auto now_s = time.seconds();
    bool probing = (now_s - first_read_time_.seconds()) < 2.0;
    for (size_t i = 0; i < motor_params_.size(); ++i) {
      // After probing, skip dead motors entirely — no CAN send,
      // no feedback wait.  This keeps the control loop fast and
      // avoids spamming the CAN bus with frames to nonexistent
      // motor IDs, which can cause bus errors that interfere
      // with the real motor's communication.
      if (!probing && !motor_alive_[i]) continue;

      // Apply motor_direction: negate command & feedforward for reversed motors
      float p_des = static_cast<float>(hw_position_commands_[i]) * motor_params_[i].motor_direction;
      float v_des = motor_params_[i].vel_max * 0.3f * motor_params_[i].motor_direction;

      // ── Debug: log every CAN frame sent to real motor (0x01) ──
      static int can_log_cnt = 0;
      if (motor_params_[i].can_id == 0x01 && (++can_log_cnt % 20 == 0)) {
        uint8_t preview[8] = {};
        pack_float_le(&preview[0], p_des);
        pack_float_le(&preview[4], v_des);
        RCLCPP_INFO(logger_,
          "CAN-TX | ID=0x%03X | data=[%02X %02X %02X %02X %02X %02X %02X %02X] | p_des=%+.4f rad | v_des=%.3f rad/s",
          0x100 + motor_params_[i].can_id,
          preview[0], preview[1], preview[2], preview[3],
          preview[4], preview[5], preview[6], preview[7],
          static_cast<double>(p_des), static_cast<double>(v_des));
      }

      send_position_cmd(static_cast<uint8_t>(i), p_des, v_des);

      if (probing || motor_alive_[i]) {
        U2CanRxFrame rx;
        if (serial_recv(rx, 3)) {
          if (!motor_alive_[i]) {
            motor_alive_[i] = true;
            RCLCPP_INFO(logger_,
              "Motor CAN ID=0x%02X (joint '%s') is alive",
              motor_params_[i].can_id, info_.joints[i].name.c_str());
          }
          process_rx_frame(rx);
        }
      }
    }
    last_write_time_ = time;
  }

  // Drain remaining frames
  {
    U2CanRxFrame rx;
    while (serial_recv(rx, 0)) {
      process_rx_frame(rx);
    }
  }

  // Copy feedback to state.  Dead motors report commanded position
  // so MoveIt's planning model stays consistent with what was sent.
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (size_t i = 0; i < motor_params_.size(); ++i) {
      auto & fb = motor_feedback_[i];
      if (fb.fresh) {
        hw_position_states_[i] = fb.position;
        hw_velocity_states_[i] = fb.velocity;
        fb.fresh = false;
      } else if (motors_enabled_ && !motor_alive_[i]) {
        // Dead motor — report 0.0 so MoveIt sees a neutral configuration.
        // Previously tracked hw_position_commands_[i], which could hold a
        // stale non-zero value from a prior trajectory, causing MoveIt to
        // see impossible joint angles (e.g. joint3 at 12.4 rad) and fail
        // collision checks / distort the RViz model.
        hw_position_states_[i] = 0.0;
        hw_velocity_states_[i] = 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════
//  write() — log and update (commands arrive via /position_commands)
// ═══════════════════════════════════════════════════════════════════

hardware_interface::return_type DmMotorHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  bool log_now = (++write_count_ % LOG_THROTTLE == 0);

  if (log_now && motors_enabled_) {
    for (size_t i = 0; i < motor_params_.size(); ++i) {
      // Only log the real motor (CAN ID 0x01) — suppress noise from dead joints
      if (motor_params_[i].can_id != 0x01) continue;
      const float p_des = static_cast<float>(hw_position_commands_[i]);
      const float v_des = motor_params_[i].vel_max;
      RCLCPP_INFO(logger_,
        "CMD | joint%zu | CAN ID=0x%03X | p_des=%+7.3f rad (%+6.1f°) | v_des=%.1f rad/s",
        i, 0x100 + motor_params_[i].can_id,
        static_cast<double>(p_des), static_cast<double>(p_des) * 180.0 / M_PI,
        static_cast<double>(v_des));
    }
  }

  last_write_time_ = time;
  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════
//  Feedback processing
// ═══════════════════════════════════════════════════════════════════

void DmMotorHardware::process_rx_frame(const U2CanRxFrame &rx)
{
  // Only process success frames
  if (rx.cmd != 0x11 || rx.frame_end != 0x55) return;

  const uint8_t *data = rx.can_data;

  // The CAN ID in feedback is the motor's slave ID
  // (motor_id + POS_MODE offset is stripped by the motor firmware)
  // or it's 0x00 when master_id=0, in which case motor_id is in data[0] low nibble

  uint8_t motor_id;
  if (rx.can_id != 0x00) {
    motor_id = rx.can_id & 0xFF;
  } else {
    motor_id = data[0] & 0x0F;
  }

  // Find motor index by CAN ID
  int motor_idx = -1;
  for (size_t i = 0; i < motor_params_.size(); ++i) {
    if (motor_params_[i].can_id == motor_id) {
      motor_idx = static_cast<int>(i);
      break;
    }
  }
  if (motor_idx < 0) return;  // not our motor

  uint8_t error_flags = (data[0] >> 4) & 0x0F;

  // Decode position (uint16 per DM protocol: maps [0,65535] → [pos_min, pos_max])
  uint16_t raw_pos = static_cast<uint16_t>(data[1]) << 8 | data[2];
  double pos = decode_position(motor_idx, raw_pos);

  // Decode velocity (int12)
  int16_t raw_vel = extract_int12(data[3], data[4] >> 4);
  double vel = decode_velocity(motor_idx, raw_vel);

  // Decode torque (int12)
  int16_t raw_trq = extract_int12_high_nibble(data[4] & 0x0F, data[5]);
  double trq = decode_torque(motor_idx, raw_trq);

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    auto &fb = motor_feedback_[motor_idx];
    // Apply motor_direction: negate feedback for reversed motors
    fb.position    = pos * motor_params_[motor_idx].motor_direction;
    fb.velocity    = vel * motor_params_[motor_idx].motor_direction;
    fb.torque      = trq;
    fb.error_flags = error_flags;
    fb.fresh       = true;

    // ── Debug: log feedback from real motor (0x01) ──
    static int fb_log_cnt = 0;
    if (motor_params_[motor_idx].can_id == 0x01 && (++fb_log_cnt % 20 == 0)) {
      RCLCPP_INFO(logger_,
        "CAN-RX | ID=0x%03X | raw_data=[%02X %02X %02X %02X %02X %02X %02X %02X] | pos=%+.4f rad (%+.1f°) | vel=%.3f rad/s | err=0x%X",
        0x100 + motor_params_[motor_idx].can_id,
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7],
        pos, pos * 180.0 / M_PI, vel, error_flags);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  Feedback decoding (int16 / int12 → float rad)
// ═══════════════════════════════════════════════════════════════════

double DmMotorHardware::decode_position(uint8_t motor_index, uint16_t raw) const
{
  if (motor_index >= motor_params_.size()) return 0.0;
  const auto &p = motor_params_[motor_index];
  // DM protocol: uint16 [0, 65535] maps linearly to [pos_min, pos_max]
  double span = static_cast<double>(p.pos_max - p.pos_min);
  return (static_cast<double>(raw) / 65535.0) * span + static_cast<double>(p.pos_min);
}

double DmMotorHardware::decode_velocity(uint8_t motor_index, int16_t raw) const
{
  if (motor_index >= motor_params_.size()) return 0.0;
  return (static_cast<double>(raw) / 2048.0) * motor_params_[motor_index].vel_max;
}

double DmMotorHardware::decode_torque(uint8_t motor_index, int16_t raw) const
{
  if (motor_index >= motor_params_.size()) return 0.0;
  return (static_cast<double>(raw) / 2048.0) * motor_params_[motor_index].trq_max;
}

// ═══════════════════════════════════════════════════════════════════
//  Static utilities
// ═══════════════════════════════════════════════════════════════════

void DmMotorHardware::pack_float_le(uint8_t *buf, float val)
{
  std::memcpy(buf, &val, sizeof(float));
}

int16_t DmMotorHardware::extract_int12(uint8_t hi_byte, uint8_t lo_nibble)
{
  int16_t raw = (static_cast<int16_t>(hi_byte) << 4) | (lo_nibble & 0x0F);
  if (raw & 0x0800) raw |= 0xF000;
  return raw;
}

int16_t DmMotorHardware::extract_int12_high_nibble(uint8_t hi_nibble, uint8_t lo_byte)
{
  int16_t raw = (static_cast<int16_t>(hi_nibble & 0x0F) << 8) | lo_byte;
  if (raw & 0x0800) raw |= 0xF000;
  return raw;
}

}  // namespace urdfyong_hardware

PLUGINLIB_EXPORT_CLASS(
  urdfyong_hardware::DmMotorHardware,
  hardware_interface::SystemInterface)
