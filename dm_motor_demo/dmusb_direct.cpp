// dmusb_direct.cpp — Direct libusb access to DM USB2CANFD (VID 2e88 PID 4603)
//
// This bypasses the DM SDK entirely. It detaches the cdc_acm kernel driver,
// claims the USB interfaces, and communicates via bulk endpoints.
//
// Build:
//   g++ -std=c++17 -O2 dmusb_direct.cpp -o dmusb_direct \
//       -I../dm-device-sdk/sdk-lib/linux/x86_64/include \
//       -L../dm-device-sdk/sdk-lib/linux/x86_64/lib -lusb-1.0 \
//       -Wl,-rpath,../dm-device-sdk/sdk-lib/linux/x86_64/lib
//
// Run with sudo (required to detach kernel driver):
//   sudo ./dmusb_direct

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <libusb-1.0/libusb.h>

// ── Config ─────────────────────────────────────────────────────────
static constexpr uint16_t TARGET_VID  = 0x2e88;
static constexpr uint16_t TARGET_PID  = 0x4603;
static constexpr uint8_t  MOTOR_ID    = 0x01;
static constexpr uint32_t CAN_ID_BASE = 0x100;

// Special CAN command payloads
static const uint8_t CMD_ENABLE[8]      = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t CMD_DISABLE[8]     = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t CMD_SET_ZERO[8]    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
static const uint8_t CMD_CLEAR_ERROR[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

// ── Global state ────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};
static libusb_device_handle *g_handle = nullptr;
static bool g_kernel_detached = false;

// ── Helpers ─────────────────────────────────────────────────────────
static void pack_float_le(uint8_t *buf, float val) {
    std::memcpy(buf, &val, sizeof(float));
}

// ── Signal handler ──────────────────────────────────────────────────
static void sig_handler(int) {
    std::cout << "\n[!] Signal received, shutting down...\n";
    g_running = false;
}

// ══════════════════════════════════════════════════════════════════════
//  CAN Frame format discovery
//
//  We don't know the exact USB protocol the DM USB2CANFD uses.
//  This program tries several common formats:
//    Format A: Raw 8-byte payload (CAN ID inferred from USB endpoint)
//    Format B: [can_id LE 4B][dlc 1B][payload...]
//    Format C: DM SDK usb_rx_frame_t header + payload
//    Format D: slcan ASCII
// ══════════════════════════════════════════════════════════════════════

enum class CanFormat { RAW, HEADER_B, SDK_HEADER, SLCAN };

// Send a CAN frame using the specified format
static bool send_can_frame(CanFormat fmt, uint32_t can_id,
                           const uint8_t *data, uint8_t dlen) {
    if (!g_handle) return false;

    uint8_t buf[128];
    int buflen = 0;
    int transferred = 0;

    switch (fmt) {
    case CanFormat::RAW:
        // Just send the raw payload — CAN ID must be configured elsewhere
        if (dlen > 64) return false;
        std::memcpy(buf, data, dlen);
        buflen = dlen;
        break;

    case CanFormat::HEADER_B: {
        // [CAN ID LE 4B][DLC 1B][Data 0-8B]
        buf[0] = can_id & 0xFF;
        buf[1] = (can_id >> 8) & 0xFF;
        buf[2] = (can_id >> 16) & 0xFF;
        buf[3] = (can_id >> 24) & 0xFF;
        buf[4] = dlen;
        std::memcpy(buf + 5, data, dlen);
        buflen = 5 + dlen;
        break;
    }

    case CanFormat::SDK_HEADER: {
        // DM SDK format (guessed from usb_rx_frame_head_t):
        // [can_id 4B LE][flags 1B][channel 1B][dlc 1B][reserved 3B][payload...]
        // flags: bit0=canfd, bit1=ext, bit2=rtr, bit3=brs
        uint32_t id = can_id & 0x1FFFFFFF;
        buf[0] = id & 0xFF;
        buf[1] = (id >> 8) & 0xFF;
        buf[2] = (id >> 16) & 0xFF;
        buf[3] = (id >> 24) & 0xFF;
        buf[4] = 0x00;  // flags: canfd=0, ext=0, rtr=0, brs=0
        buf[5] = 0x00;  // channel
        buf[6] = dlen;  // dlc
        buf[7] = 0x00;  // reserved
        buf[8] = 0x00;  // reserved (time_stamp placeholder for tx)
        buf[9] = 0x00;  // reserved
        std::memcpy(buf + 10, data, dlen);
        buflen = 10 + dlen;
        break;
    }

    case CanFormat::SLCAN: {
        // SLCAN ASCII: "tIIILD...\r" for standard frames
        char tmp[64];
        int off = snprintf(tmp, sizeof(tmp), "t%03X%d", can_id & 0x7FF, dlen);
        for (int i = 0; i < dlen; i++) {
            off += snprintf(tmp + off, sizeof(tmp) - off, "%02X", data[i]);
        }
        tmp[off++] = '\r';
        std::memcpy(buf, tmp, off);
        buflen = off;
        break;
    }
    }

    // Send via bulk OUT endpoint (EP 5 OUT)
    int rc = libusb_bulk_transfer(g_handle, 0x05, buf, buflen, &transferred, 1000);
    if (rc < 0) {
        std::cerr << "[TX] Bulk transfer failed: " << libusb_error_name(rc) << "\n";
        return false;
    }

    std::cout << "[TX] " << transferred << " bytes sent (fmt="
              << (int)fmt << ")\n";
    return true;
}

// Receive thread — poll bulk IN endpoint (EP 4 IN)
static void recv_thread_func() {
    uint8_t buf[64];
    int transferred = 0;

    while (g_running) {
        int rc = libusb_bulk_transfer(g_handle, 0x84, buf, sizeof(buf),
                                      &transferred, 500);
        if (rc == LIBUSB_ERROR_TIMEOUT) continue;
        if (rc < 0) {
            if (rc == LIBUSB_ERROR_NO_DEVICE) break;
            std::cerr << "[RX] Error: " << libusb_error_name(rc) << "\n";
            continue;
        }

        std::cout << "[RX] " << transferred << " bytes: ";
        for (int i = 0; i < transferred && i < 64; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << (int)buf[i] << " ";
        }
        std::cout << std::dec << "\n" << std::flush;
    }
}

// ══════════════════════════════════════════════════════════════════════
//  Main
// ══════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::cout << "=== DM USB2CANFD Direct Access Test ===\n";

    // ── Init libusb ────────────────────────────────────────────────
    libusb_context *ctx = nullptr;
    int rc = libusb_init(&ctx);
    if (rc < 0) {
        std::cerr << "libusb_init failed: " << libusb_error_name(rc) << "\n";
        return 1;
    }
    libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_WARNING);

    // ── Find our device ────────────────────────────────────────────
    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    libusb_device *found = nullptr;

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        libusb_get_device_descriptor(devs[i], &desc);
        if (desc.idVendor == TARGET_VID && desc.idProduct == TARGET_PID) {
            found = devs[i];
            std::cout << "[+] Found device: VID=0x" << std::hex
                      << desc.idVendor << " PID=0x" << desc.idProduct
                      << std::dec << " (bus="
                      << (int)libusb_get_bus_number(devs[i])
                      << " addr=" << (int)libusb_get_device_address(devs[i])
                      << ")\n";
            break;
        }
    }

    if (!found) {
        std::cerr << "[-] Device not found. Check USB connection.\n";
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return 1;
    }

    // ── Open device ────────────────────────────────────────────────
    rc = libusb_open(found, &g_handle);
    if (rc < 0) {
        std::cerr << "libusb_open failed: " << libusb_error_name(rc) << "\n"
                  << "  Run with sudo!\n";
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return 1;
    }
    libusb_free_device_list(devs, 1);
    std::cout << "[+] Device opened.\n";

    // ── Detach kernel driver (cdc_acm) ────────────────────────────
    // The kernel's cdc_acm driver is attached to interfaces 0 and 1.
    // We need to detach it to claim the interfaces ourselves.
    for (int iface = 0; iface <= 1; iface++) {
        if (libusb_kernel_driver_active(g_handle, iface) == 1) {
            rc = libusb_detach_kernel_driver(g_handle, iface);
            if (rc < 0) {
                std::cerr << "[-] Failed to detach kernel driver from iface "
                          << iface << ": " << libusb_error_name(rc) << "\n";
            } else {
                std::cout << "[+] Detached kernel driver from interface "
                          << iface << "\n";
                g_kernel_detached = true;
            }
        }
    }

    // ── Claim interfaces ──────────────────────────────────────────
    for (int iface = 0; iface <= 1; iface++) {
        rc = libusb_claim_interface(g_handle, iface);
        if (rc < 0) {
            std::cerr << "[-] Failed to claim interface " << iface
                      << ": " << libusb_error_name(rc) << "\n";
        } else {
            std::cout << "[+] Claimed interface " << iface << "\n";
        }
    }

    // ── Set up alternate interface if needed ──────────────────────
    // CDC Data interface (1) might need altsetting 0
    rc = libusb_set_interface_alt_setting(g_handle, 1, 0);
    if (rc < 0) {
        std::cerr << "[-] set_interface_alt_setting(1,0) failed: "
                  << libusb_error_name(rc) << "\n";
    }

    std::cout << "\n[READY] Sending test frames...\n\n";

    // ── Start receive thread ──────────────────────────────────────
    std::thread recv_thread(recv_thread_func);

    // ── Try each format to discover the protocol ──────────────────
    const uint32_t can_id = CAN_ID_BASE + MOTOR_ID;  // 0x101

    // First try SLCAN (ASCII protocol)
    std::cout << "--- Trying SLCAN format ---\n";
    for (int i = 0; i < 3; i++) {
        // SLCAN open command
        const uint8_t open_cmd[] = {'O', '\r'};
        int transferred;
        libusb_bulk_transfer(g_handle, 0x05,
                             const_cast<uint8_t*>(open_cmd), 2,
                             &transferred, 500);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    send_can_frame(CanFormat::SLCAN, can_id, CMD_ENABLE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Try HEADER_B format
    std::cout << "--- Trying HEADER_B format ---\n";
    send_can_frame(CanFormat::HEADER_B, can_id, CMD_ENABLE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Try SDK_HEADER format
    std::cout << "--- Trying SDK_HEADER format ---\n";
    send_can_frame(CanFormat::SDK_HEADER, can_id, CMD_ENABLE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Try RAW format (just the payload)
    std::cout << "--- Trying RAW format ---\n";
    send_can_frame(CanFormat::RAW, can_id, CMD_ENABLE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── Try via control transfer (endpoint 0) ─────────────────────
    std::cout << "--- Trying control transfer ---\n";
    {
        uint8_t ctrl_buf[64];
        // Send CAN frame via vendor control request
        rc = libusb_control_transfer(g_handle,
            0x40,       // bmRequestType: host-to-device, vendor, interface
            0x01,       // bRequest (guess)
            can_id,     // wValue
            0,          // wIndex
            const_cast<uint8_t*>(CMD_ENABLE), 8,  // data
            1000);
        std::cout << "  control_transfer(0x40,0x01): " << libusb_error_name(rc) << "\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── Try with interface 1 (CDC Data) specifically ──────────────
    std::cout << "--- Trying via iface 1 EP 0x05 (bulk OUT) with clear error ---\n";
    send_can_frame(CanFormat::HEADER_B, can_id, CMD_CLEAR_ERROR, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    send_can_frame(CanFormat::HEADER_B, can_id, CMD_ENABLE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── Wait for user ─────────────────────────────────────────────
    std::cout << "\n[DONE] All formats tried.\n";
    std::cout << "If the motor moved, note which format worked.\n";
    std::cout << "Press Enter to cleanup and exit...\n";
    getchar();

    // ── Cleanup ───────────────────────────────────────────────────
    g_running = false;
    recv_thread.join();

    // Re-attach kernel driver
    for (int iface = 0; iface <= 1; iface++) {
        libusb_release_interface(g_handle, iface);
        if (g_kernel_detached) {
            libusb_attach_kernel_driver(g_handle, iface);
            std::cout << "[+] Re-attached kernel driver for interface "
                      << iface << "\n";
        }
    }

    libusb_close(g_handle);
    libusb_exit(ctx);
    std::cout << "[+] Cleanup complete.\n";
    return 0;
}
