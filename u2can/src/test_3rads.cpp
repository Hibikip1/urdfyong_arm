// Quick test: 3 rad/s position-velocity control
#include "damiao.h"
#include <unistd.h>
#include <cmath>
#include <iostream>

damiao::Motor M1(damiao::DM4310, 0x01, 0x11);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

int main(int argc, char *argv[]) {
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = damiao::Motor_Control(serial);

    dm.addMotor(&M1);

    // Disable first, then switch mode
    std::cout << "[1] Disabling motor..." << std::endl;
    dm.disable(M1);
    sleep(1);

    std::cout << "[2] Switching to POS_VEL_MODE..." << std::endl;
    if (dm.switchControlMode(M1, damiao::POS_VEL_MODE))
        std::cout << "    POS_VEL_MODE OK" << std::endl;
    else
        std::cout << "    POS_VEL_MODE FAILED!" << std::endl;

    std::cout << "[3] Enabling motor..." << std::endl;
    dm.enable(M1);
    sleep(1);

    // Read current position
    dm.refresh_motor_status(M1);
    float start_pos = M1.Get_Position();
    std::cout << "[4] Current position: " << start_pos << " rad" << std::endl;

    // Target: hold current position, velocity = 3 rad/s
    float target_pos = start_pos + 1.0f;  // move 1 rad forward
    float vel = 3.0f;  // 3 rad/s

    std::cout << "[5] Moving from " << start_pos << " to " << target_pos
              << " rad at " << vel << " rad/s" << std::endl;
    std::cout << "    (Press Ctrl+C to stop)" << std::endl;

    // Control loop: send pos+vel command at ~1kHz
    auto start_time = std::chrono::steady_clock::now();
    int count = 0;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();

        // Alternate direction every 2 seconds
        float t = fmod(elapsed, 4.0f);
        if (t < 2.0f) {
            target_pos = start_pos + 1.0f;  // forward
        } else {
            target_pos = start_pos - 1.0f;  // backward
        }

        dm.control_pos_vel(M1, target_pos, vel);

        // Print status every 500 cycles (~2 Hz)
        if (count % 500 == 0) {
            dm.refresh_motor_status(M1);
            std::cout << "  t=" << elapsed << "s | pos=" << M1.Get_Position()
                      << " rad | vel=" << M1.Get_Velocity()
                      << " rad/s | torque=" << M1.Get_tau()
                      << " | err=" << M1.Get_Err() << std::endl;
        }

        usleep(1000);  // ~1ms control loop
        count++;
    }

    return 0;
}
