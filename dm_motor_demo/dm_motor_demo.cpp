// DM Motor Control Demo — standalone interactive test
//
// Controls one DM-series motor via USB2CANFD.
// Type a position (radians) + Enter to send, or use quick keys:
//   e = enable    d = disable    z = set zero
//   0 = go to 0 rad    1 = +1 rad    2 = -1 rad
//   +/- = step ±0.1 rad    q = quit
//
// Build:
//   mkdir -p build && cd build
//   cmake .. && make
//   ./dm_motor_demo
//
// Or one-liner:
//   g++ -std=c++17 -O2 dm_motor_demo.cpp \
//       -I../dm-device-sdk/sdk-lib \
//       -L../dm-device-sdk/sdk-lib/linux/x86_64 \
//       -ldm_device -lpthread -Wl,-rpath,../dm-device-sdk/sdk-lib/linux/x86_64 \
//       -o dm_motor_demo

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#include "dmcan.h"

// ═══════════════════════════════════════════════════════════════════════
//  Configuration — tweak these to match your setup
// ═══════════════════════════════════════════════════════════════════════
static constexpr uint8_t  MOTOR_CAN_ID    = 0x01;    // motor CAN ID
static constexpr uint32_t CAN_ID_BASE     = 0x100;   // position-velocity mode base
static constexpr uint32_t CAN_BAUDRATE    = 1000000; // 1 Mbps
static constexpr float    DEFAULT_VEL     = 2.0f;    // rad/s — trapezoidal speed

// Special CAN commands (8-byte payloads)
static const uint8_t CMD_ENABLE[8]      = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t CMD_DISABLE[8]     = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t CMD_SET_ZERO[8]    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
static const uint8_t CMD_CLEAR_ERROR[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

// ═══════════════════════════════════════════════════════════════════════
//  Global state (for signal handler + callback trampoline)
// ═══════════════════════════════════════════════════════════════════════
static dmcan_context *        g_ctx  = nullptr;
static dmcan_device_handle *  g_dev  = nullptr;
static std::atomic<bool>      g_running{true};
static std::atomic<bool>      g_motor_enabled{false};

// Latest feedback (thread-safe: written by recv callback, read by main)
struct MotorFeedback {
    double   position   = 0.0;   // rad
    double   velocity   = 0.0;   // rad/s
    double   torque     = 0.0;   // N·m
    uint8_t  mos_temp   = 0;     // °C
    uint8_t  coil_temp  = 0;     // °C
    uint8_t  error_flags = 0;    // 0=disabled, 1=enabled, 8+=fault
    bool     fresh      = false;
};
static MotorFeedback g_feedback;
static std::mutex    g_fb_mutex;

// Position range for decoding (match your motor's P_MIN / P_MAX)
static float g_pos_min = -12.56f;
static float g_pos_max =  12.56f;
static float g_vel_max =  30.0f;    // rad/s — for velocity decode
static float g_trq_max =  11.0f;    // N·m   — for torque decode

static float g_target_position = 0.0f;  // current command position

// ═══════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════

static void pack_float_le(uint8_t *buf, float val) {
    std::memcpy(buf, &val, sizeof(float));
}

static float unpack_float_le(const uint8_t *buf) {
    float val;
    std::memcpy(&val, buf, sizeof(float));
    return val;
}

// ═══════════════════════════════════════════════════════════════════════
//  CAN send
// ═══════════════════════════════════════════════════════════════════════

static bool send_can(uint32_t can_id, const uint8_t *data, uint8_t dlen = 8) {
    if (!g_dev) return false;
    return dmcan_device_send_can(
        g_dev, 0, can_id,
        false,  // canfd
        false,  // ext (standard frame)
        false,  // rtr
        false,  // brs
        dlen,
        const_cast<uint8_t*>(data));
}

static void send_enable() {
    // Clear errors first, then enable
    send_can(CAN_ID_BASE + MOTOR_CAN_ID, CMD_CLEAR_ERROR);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    send_can(CAN_ID_BASE + MOTOR_CAN_ID, CMD_ENABLE);
    g_motor_enabled = true;
    std::cout << "[INFO] Motor 0x" << std::hex << (int)MOTOR_CAN_ID << std::dec
              << " enabled.\n";
}

static void send_disable() {
    send_can(CAN_ID_BASE + MOTOR_CAN_ID, CMD_DISABLE);
    g_motor_enabled = false;
    std::cout << "[INFO] Motor disabled.\n";
}

static void send_set_zero() {
    send_can(CAN_ID_BASE + MOTOR_CAN_ID, CMD_SET_ZERO);
    std::cout << "[INFO] Set-zero command sent (current position = 0).\n";
}

static void send_position_cmd(float p_des, float v_des) {
    uint8_t data[8] = {};
    pack_float_le(&data[0], p_des);
    pack_float_le(&data[4], v_des);
    send_can(CAN_ID_BASE + MOTOR_CAN_ID, data, 8);
}

static void send_heartbeat() {
    // Re-send current position command to keep motor alive
    send_position_cmd(g_target_position, DEFAULT_VEL);
}

// ═══════════════════════════════════════════════════════════════════════
//  Feedback decoding
// ═══════════════════════════════════════════════════════════════════════

static double decode_position(int16_t raw) {
    double half_range = (g_pos_max - g_pos_min) / 2.0;
    double center     = (g_pos_max + g_pos_min) / 2.0;
    return center + (raw / 32768.0) * half_range;
}

static double decode_velocity(int16_t raw) {
    return (raw / 2048.0) * g_vel_max;
}

static double decode_torque(int16_t raw) {
    return (raw / 2048.0) * g_trq_max;
}

static int16_t extract_int12(uint8_t hi_byte, uint8_t lo_nibble) {
    int16_t raw = (static_cast<int16_t>(hi_byte) << 4) | (lo_nibble & 0x0F);
    if (raw & 0x0800) raw |= 0xF000;  // sign-extend from 12 bits
    return raw;
}

static int16_t extract_int12_high_nibble(uint8_t hi_nibble, uint8_t lo_byte) {
    int16_t raw = (static_cast<int16_t>(hi_nibble & 0x0F) << 8) | lo_byte;
    if (raw & 0x0800) raw |= 0xF000;
    return raw;
}

// ═══════════════════════════════════════════════════════════════════════
//  Receive callback
// ═══════════════════════════════════════════════════════════════════════

static void recv_callback(dmcan_device_handle *, usb_rx_frame_t *frame) {
    if (!frame) return;

    uint32_t can_id = frame->head.can_id & 0x1FFFFFFF;
    // Motor feedback arrives on master CAN ID (0x00)
    if (can_id != 0) return;

    const uint8_t *p = frame->payload;

    uint8_t motor_idx = p[0] & 0x0F;
    // Only process our motor
    if (motor_idx != MOTOR_CAN_ID) return;

    uint8_t error_flags = (p[0] >> 4) & 0x0F;

    // Position
    int16_t raw_pos = static_cast<int16_t>(
        (static_cast<uint16_t>(p[1]) << 8) | p[2]);
    double pos = decode_position(raw_pos);

    // Velocity
    int16_t raw_vel = extract_int12(p[3], p[4] & 0x0F);
    double vel = decode_velocity(raw_vel);

    // Torque
    int16_t raw_trq = extract_int12_high_nibble(p[4] >> 4, p[5]);
    double trq = decode_torque(raw_trq);

    uint8_t mos_temp  = p[6];
    uint8_t coil_temp = p[7];

    {
        std::lock_guard<std::mutex> lock(g_fb_mutex);
        g_feedback.position    = pos;
        g_feedback.velocity    = vel;
        g_feedback.torque      = trq;
        g_feedback.mos_temp    = mos_temp;
        g_feedback.coil_temp   = coil_temp;
        g_feedback.error_flags = error_flags;
        g_feedback.fresh       = true;
    }

    // Print immediately
    const char* state = (error_flags == 0) ? "DISABLED"
                      : (error_flags == 1) ? "ENABLED"
                      : "FAULT";
    std::cout << "\r[FB] pos=" << std::fixed << std::setprecision(3) << pos
              << " rad (" << std::setprecision(1) << pos * 180.0 / M_PI << "°)"
              << " | vel=" << std::setprecision(2) << vel << " rad/s"
              << " | trq=" << std::setprecision(2) << trq << " N·m"
              << " | MOS=" << (int)mos_temp << "°C"
              << " coil=" << (int)coil_temp << "°C"
              << " | " << state
              << "          " << std::flush;
}

// ═══════════════════════════════════════════════════════════════════════
//  Signal handler
// ═══════════════════════════════════════════════════════════════════════

static void signal_handler(int) {
    std::cout << "\n[SIGNAL] Shutting down...\n";
    g_running = false;
}

// ═══════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Allow overriding motor CAN ID from command line
    uint8_t motor_id = MOTOR_CAN_ID;
    if (argc > 1) {
        int id = std::stoi(argv[1]);
        if (id >= 1 && id <= 0x7F) {
            motor_id = static_cast<uint8_t>(id);
        } else {
            std::cerr << "Invalid motor ID: " << id << " (must be 1-127)\n";
            return 1;
        }
    }

    std::cout << R"(
╔══════════════════════════════════════════════════════╗
║       DM Motor Control Demo — USB2CANFD             ║
╠══════════════════════════════════════════════════════╣
║  Motor CAN ID: 0x)"
              << std::hex << std::setw(2) << std::setfill('0') << (int)motor_id
              << std::dec << R"(                                   ║
║  Commands:                                          ║
║    <number> + Enter  = go to position (rad)         ║
║    e = enable motor                                 ║
║    d = disable motor                                ║
║    z = set current position as zero                 ║
║    0 = go to 0 rad                                  ║
║    1 = go to +1 rad                                 ║
║    2 = go to -1 rad                                 ║
║    + = step +0.1 rad                                ║
║    - = step -0.1 rad                                ║
║    q = quit                                         ║
╚══════════════════════════════════════════════════════╝
)" << std::endl;

    // ── Step 1: Create DM context ──────────────────────────────────
    std::cout << "[INIT] Creating DM device context...\n";
    dmcan_context_create(&g_ctx);
    if (!g_ctx) {
        std::cerr << "[FATAL] Failed to create DM context.\n";
        return 1;
    }
    dmcan_print_version(g_ctx);

    // ── Step 2: Find devices ───────────────────────────────────────
    int dev_cnt = dmcan_find_devices(g_ctx);
    std::cout << "[INIT] Found " << dev_cnt << " DM device(s)\n";
    if (dev_cnt <= 0) {
        std::cerr << "[FATAL] No USB2CANFD device found. Check USB connection.\n";
        dmcan_context_destroy(g_ctx);
        return 1;
    }

    dmcan_show_all_devices(g_ctx);

    // ── Step 3: Open device ────────────────────────────────────────
    if (!dmcan_device_get(g_ctx, &g_dev, 0)) {
        std::cerr << "[FATAL] Failed to get device at index 0.\n";
        dmcan_context_destroy(g_ctx);
        return 1;
    }

    if (!dmcan_device_open(g_dev)) {
        std::cerr << "[FATAL] Failed to open device.\n"
                  << "        Check USB permissions (try: sudo chmod 666 /dev/ttyUSB*)\n";
        dmcan_context_destroy(g_ctx);
        return 1;
    }
    std::cout << "[INIT] Device opened.\n";
    dmcan_device_print_version(g_dev);

    // ── Step 4: Enable CAN channel ─────────────────────────────────
    if (!dmcan_device_enable_channel(g_dev, 0)) {
        std::cerr << "[FATAL] Failed to enable CAN channel 0.\n";
        dmcan_device_close(g_dev);
        dmcan_context_destroy(g_ctx);
        return 1;
    }

    dmcan_channel_can_info_t can_info = {};
    can_info.channel        = 0;
    can_info.canfd          = false;
    can_info.can_baudrate   = CAN_BAUDRATE;
    can_info.canfd_baudrate = 0;
    can_info.can_sp         = 0.75f;
    can_info.canfd_sp       = 0.0f;

    if (!dmcan_device_set_channel_baudrate(g_dev, 0, can_info)) {
        std::cerr << "[FATAL] Failed to set CAN baudrate.\n";
        dmcan_device_close(g_dev);
        dmcan_context_destroy(g_ctx);
        return 1;
    }
    std::cout << "[INIT] CAN channel 0 @ " << CAN_BAUDRATE << " bps\n";

    // ── Step 5: Register receive callback ──────────────────────────
    dmcan_device_hook_recv_callback(g_dev, recv_callback);
    std::cout << "[INIT] Receive callback registered.\n";

    // ── Step 6: Send initial query (heartbeat) ─────────────────────
    std::cout << "[INIT] Sending initial heartbeat to motor 0x"
              << std::hex << (int)motor_id << std::dec << "...\n";
    send_position_cmd(0.0f, 0.0f);

    // ── Step 7: Enable motor ───────────────────────────────────────
    std::cout << "[INIT] Enabling motor...\n";
    send_can(CAN_ID_BASE + motor_id, CMD_CLEAR_ERROR);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    send_can(CAN_ID_BASE + motor_id, CMD_ENABLE);
    g_motor_enabled = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::cout << "\n[MOTOR] Ready! Enter commands (q to quit):\n\n";

    // ═══════════════════════════════════════════════════════════════
    //  Main loop — heartbeat thread + interactive input
    // ═══════════════════════════════════════════════════════════════

    // Heartbeat thread: re-send position command every 20ms so motor
    // stays enabled and we keep getting feedback.
    std::thread heartbeat([&]() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (g_motor_enabled) {
                send_position_cmd(g_target_position, DEFAULT_VEL);
            } else {
                // Even when disabled, send zero-velocity to trigger feedback
                send_position_cmd(0.0f, 0.0f);
            }
        }
    });

    // ── Interactive input loop ─────────────────────────────────────
    std::string line;
    while (g_running && std::getline(std::cin, line)) {
        if (line.empty()) continue;

        char cmd = line[0];

        if (cmd == 'q' || cmd == 'Q') {
            break;
        }
        else if (cmd == 'e' || cmd == 'E') {
            send_enable();
        }
        else if (cmd == 'd' || cmd == 'D') {
            send_disable();
        }
        else if (cmd == 'z' || cmd == 'Z') {
            send_set_zero();
        }
        else if (cmd == '+' || line == "=") {
            g_target_position += 0.1f;
            std::cout << "-> target = " << g_target_position << " rad\n";
        }
        else if (cmd == '-') {
            g_target_position -= 0.1f;
            std::cout << "-> target = " << g_target_position << " rad\n";
        }
        else {
            // Try to parse as a number
            try {
                float val = std::stof(line);
                g_target_position = val;
                std::cout << "-> target = " << g_target_position
                          << " rad (" << g_target_position * 180.0 / M_PI << "°)\n";
            } catch (...) {
                std::cout << "[?] Unknown command: '" << line << "'\n"
                          << "    e=enable  d=disable  z=set-zero  q=quit  "
                          << "+/-=step  0/1/2=preset  <number>=position\n";
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  Cleanup
    // ═══════════════════════════════════════════════════════════════

    std::cout << "\n[CLEANUP] Shutting down...\n";

    g_running = false;
    heartbeat.join();

    // Disable motor
    std::cout << "[CLEANUP] Disabling motor...\n";
    send_can(CAN_ID_BASE + motor_id, CMD_DISABLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Close device
    if (g_dev) {
        dmcan_device_close(g_dev);
        g_dev = nullptr;
    }
    if (g_ctx) {
        dmcan_context_destroy(g_ctx);
        g_ctx = nullptr;
    }

    std::cout << "[CLEANUP] Done.\n";
    return 0;
}
