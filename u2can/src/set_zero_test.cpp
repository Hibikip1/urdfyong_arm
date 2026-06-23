// Set Zero test — verify motor zero position
// Usage: ./set_zero_test
//
// Procedure:
//   1. Disable motor
//   2. Read current position (before zero)
//   3. Send Set Zero command (0xFE)
//   4. Read position again (should be ~0)
//   5. Save to flash (persistent) — this internally disables the motor!
//   6. Switch to POS_VEL_MODE (REQUIRED after save!)
//   7. Enable motor
//   8. Send small position command, verify tracking
#include "damiao.h"
#include <unistd.h>
#include <iostream>
#include <iomanip>

damiao::Motor M1(damiao::DM4310, 0x01, 0x11);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

int main(int argc, char *argv[]) {
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = damiao::Motor_Control(serial);
    dm.addMotor(&M1);

    std::cout << "=== DM4310 Set Zero Test ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    // ── Step 1: Disable ──────────────────────────────────────
    std::cout << "\n[1/7] Disabling motor..." << std::endl;
    dm.disable(M1);
    sleep(1);

    // ── Step 2: Read current position BEFORE zero ────────────
    dm.refresh_motor_status(M1);
    float pos_before = M1.Get_Position();
    std::cout << "[2/7] Position BEFORE Set Zero: "
              << pos_before << " rad  ("
              << pos_before * 180.0 / M_PI << "°)" << std::endl;

    // ── Step 3: Set Zero ─────────────────────────────────────
    std::cout << "[3/7] Sending Set Zero command (0xFE)..." << std::endl;
    dm.set_zero_position(M1);
    usleep(200000);  // 200ms wait

    // ── Step 4: Read position AFTER zero ─────────────────────
    dm.refresh_motor_status(M1);
    float pos_after = M1.Get_Position();
    std::cout << "[4/7] Position AFTER Set Zero:  "
              << pos_after << " rad  ("
              << pos_after * 180.0 / M_PI << "°)";

    if (std::abs(pos_after) < 0.1) {
        std::cout << "  ✅ Near zero!" << std::endl;
    } else if (std::abs(pos_after) < 1.0) {
        std::cout << "  ⚠️  Within 1 rad, but not precise" << std::endl;
    } else {
        std::cout << "  ❌ Set Zero did NOT take effect!" << std::endl;
    }

    // ── Step 5: Save to flash ────────────────────────────────
    std::cout << "[5/8] Saving to flash (persistent)... may take 2-3 seconds" << std::endl;
    dm.save_motor_param(M1);
    sleep(3);

    // ── Step 6: Switch to POS_VEL_MODE (required after save!) ─
    std::cout << "[6/8] Switching to POS_VEL_MODE..." << std::endl;
    if (dm.switchControlMode(M1, damiao::POS_VEL_MODE))
        std::cout << "      POS_VEL_MODE OK" << std::endl;
    else
        std::cout << "      POS_VEL_MODE FAILED!" << std::endl;

    // ── Step 7: Enable motor ─────────────────────────────────
    std::cout << "[7/8] Enabling motor..." << std::endl;
    dm.enable(M1);
    sleep(1);

    // ── Step 8: Verify tracking ──────────────────────────────
    std::cout << "[8/8] Testing position tracking..." << std::endl;

    // Read starting position for reference
    dm.refresh_motor_status(M1);
    float pos_start = M1.Get_Position();
    std::cout << "      Start position: " << pos_start << " rad" << std::endl;

    // Move to +0.5 rad at 2 rad/s — long enough to actually reach target
    float target = 0.5f;
    std::cout << "      Moving to +" << target << " rad at 2 rad/s..." << std::endl;

    // Send commands for ~2 seconds (more than enough at 2 rad/s)
    for (int i = 0; i < 200; i++) {
        dm.control_pos_vel(M1, target, 2.0f);
        usleep(10000);  // 10ms → 100 Hz, ~2s total
    }
    usleep(500000);  // let motor settle

    dm.refresh_motor_status(M1);
    float pos_final = M1.Get_Position();
    float error = target - pos_final;
    std::cout << "      Target: +" << target << " rad" << std::endl;
    std::cout << "      Actual: " << pos_final << " rad  ("
              << pos_final * 180.0 / M_PI << "°)" << std::endl;
    std::cout << "      Error:  " << error << " rad";

    if (std::abs(error) < 0.2) {
        std::cout << "  ✅ Good tracking!" << std::endl;
    } else if (std::abs(pos_final - pos_start) < 0.05) {
        std::cout << "  ❌ Motor did NOT move at all!" << std::endl;
    } else {
        std::cout << "  ⚠️  Motor moved but didn't reach target" << std::endl;
    }

    // ── Return to zero and disable ───────────────────────────
    std::cout << "\n[Cleanup] Returning to zero and disabling..." << std::endl;
    for (int i = 0; i < 100; i++) {
        dm.control_pos_vel(M1, 0.0f, 2.0f);
        usleep(10000);
    }
    usleep(500000);
    dm.disable(M1);

    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
