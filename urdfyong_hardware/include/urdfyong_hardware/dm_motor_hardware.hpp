// Copyright 2026 hibikip
//
// Licensed under the Apache License, Version 2.0 (the "License");
#ifndef URDFYONG_HARDWARE__DM_MOTOR_HARDWARE_HPP_
#define URDFYONG_HARDWARE__DM_MOTOR_HARDWARE_HPP_

#include <atomic>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <queue>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace urdfyong_hardware
{

// ── u2can serial protocol (u2can = USB-to-CAN via CDC ACM serial) ─

#pragma pack(push, 1)

/// TX frame: host → USB2CANFD
struct U2CanTxFrame
{
  uint8_t  frame_header[2] = {0x55, 0xAA};
  uint8_t  frame_len    = 0x1E;   // total bytes = 30
  uint8_t  cmd          = 0x03;   // 3 = non-feedback CAN forward
  uint32_t send_times   = 1;      // number of CAN frames to send
  uint32_t interval_ms  = 0;      // gap between repeated sends (ms)
  uint8_t  id_type      = 0;      // 0=standard frame, 1=extended
  uint32_t can_id       = 0;
  uint8_t  frame_type   = 0;      // 0=data, 1=remote
  uint8_t  dlc          = 8;
  uint8_t  id_acc       = 0;
  uint8_t  data_acc     = 0;
  uint8_t  data[8]      = {};
  uint8_t  crc          = 0;

  void set_can_frame(uint32_t id, const uint8_t *payload, uint8_t len = 8)
  {
    can_id = id;  dlc = len;  id_acc = 0;  data_acc = 0;
    std::memset(data, 0, sizeof(data));
    if (payload) std::memcpy(data, payload, std::min<uint8_t>(len, 8));
  }
};
static_assert(sizeof(U2CanTxFrame) == 30, "U2CanTxFrame must be 30 bytes");

/// RX frame: USB2CANFD → host
struct U2CanRxFrame
{
  uint8_t  frame_header;   // 0xAA
  uint8_t  cmd;            // 0x11 = success, 0x01 = rx fail, 0x02 = tx fail, 0xEE = error
  uint8_t  flags;          // canDataLen:6 | canIde:1 | canRtr:1
  uint32_t can_id;
  uint8_t  can_data[8];
  uint8_t  frame_end;      // 0x55

  uint8_t dlc()     const { return flags & 0x3F; }
  bool    is_ext()   const { return (flags >> 6) & 1; }
  bool    is_remote() const { return (flags >> 7) & 1; }
};
static_assert(sizeof(U2CanRxFrame) == 16, "U2CanRxFrame must be 16 bytes");

#pragma pack(pop)


// ── Hardware class ──────────────────────────────────────────────

class DmMotorHardware : public hardware_interface::SystemInterface
{
public:
  DmMotorHardware() = default;
  ~DmMotorHardware() override;

  // ── Lifecycle callbacks ──────────────────────────────────────
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ─────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Real-time read / write ───────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Per-motor configuration (from URDF <param> tags) ────────
  struct MotorParams
  {
    uint8_t can_id = 0;
    float motor_direction = 1.0f;  // +1=normal, -1=reverse (motor vs URDF convention)
    float pos_min = -12.5f;   // rad — lower bound for uint16 mapping
    float pos_max =  12.5f;   // rad — upper bound for uint16 mapping
    float vel_max =  30.0f;   // rad/s — upper bound for int12 mapping
    float trq_max =  10.0f;   // Nm — upper bound for int12 mapping
  };

  // ── Decoded feedback ────────────────────────────────────────
  struct MotorFeedback
  {
    double position = 0.0;
    double velocity = 0.0;
    double torque   = 0.0;
    uint8_t error_flags = 0;   // 0=disabled, 1=enabled, 8+=fault
    bool fresh = false;
  };

  // ── Protocol constants ───────────────────────────────────────
  static constexpr uint32_t CAN_ID_POS   = 0x100;   // position-velocity mode offset
  static constexpr uint32_t CAN_ID_REG   = 0x7FF;   // register read/write

  static constexpr uint8_t CMD_ENABLE[8]      = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
  static constexpr uint8_t CMD_DISABLE[8]     = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
  static constexpr uint8_t CMD_SET_ZERO[8]    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
  static constexpr uint8_t CMD_CLEAR_ERROR[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

  // ── Serial I/O ───────────────────────────────────────────────
  bool serial_open(const std::string &port, int baudrate);
  void serial_close();
  bool serial_send(const U2CanTxFrame &frame);
  bool serial_recv(U2CanRxFrame &frame, int timeout_ms = 2);

  // ── CAN send helpers ─────────────────────────────────────────
  void send_enable(uint8_t motor_index);
  void send_disable(uint8_t motor_index);
  void send_position_cmd(uint8_t motor_index, float p_des, float v_des);
  void send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t dlen = 8);

  // ── Feedback decoding ────────────────────────────────────────
  void process_rx_frame(const U2CanRxFrame &rx);
  double decode_position(uint8_t motor_index, uint16_t raw) const;
  double decode_velocity(uint8_t motor_index, int16_t raw) const;
  double decode_torque(uint8_t motor_index, int16_t raw) const;

  static void pack_float_le(uint8_t *buf, float val);
  static int16_t extract_int12(uint8_t hi_byte, uint8_t lo_nibble);
  static int16_t extract_int12_high_nibble(uint8_t hi_nibble, uint8_t lo_byte);

  // ── Serial port state ────────────────────────────────────────
  int serial_fd_ = -1;
  std::string serial_port_ = "/dev/ttyACM0";
  int serial_baud_ = 921600;  // B921600
  std::mutex serial_mutex_;   // guards send/recv
  std::queue<uint8_t> rx_queue_;

  // ── Configuration ────────────────────────────────────────────
  std::vector<MotorParams> motor_params_;
  std::vector<MotorFeedback> motor_feedback_;
  std::mutex feedback_mutex_;

  // ── State / command storage ──────────────────────────────────
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_position_commands_;

  // ── Runtime state ────────────────────────────────────────────
  rclcpp::Time last_write_time_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> motors_enabled_{false};
  std::vector<bool> motor_alive_;  // per-motor: has ever responded
  std::atomic<bool> device_open_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("DmMotorHardware")};
  bool first_read_ = true;
  rclcpp::Time first_read_time_{0, 0, RCL_ROS_TIME};
  int write_count_ = 0;
  static constexpr int LOG_THROTTLE = 50;

  // ── External command bridge (bypasses Humble JTC param bug) ──
  static DmMotorHardware * instance_;
  static std::shared_ptr<rclcpp::Node> g_cmd_node_;
  static std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>> g_cmd_sub_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> g_cmd_executor_;
  static std::thread g_cmd_thread_;
  static void start_cmd_bridge();
  static void stop_cmd_bridge();
};

}  // namespace urdfyong_hardware

#endif  // URDFYONG_HARDWARE__DM_MOTOR_HARDWARE_HPP_
