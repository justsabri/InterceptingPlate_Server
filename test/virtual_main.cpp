#include "virtual_devices.h"
#include <iostream>

int main() {
    VirtualMotor motor("../test/virtual_motor.json", "can0");
    VirtualImu imu("../test/virtual_imu.json");

    std::string motor_action = "启动";
    std::string imu_action = "启动";
    while (true) {
        std::cout << "\n===== 菜单 =====\n";
        std::cout << " 1)" << motor_action << "虚拟电机 " << "\n";
        std::cout << " 2)" << imu_action << "虚拟惯导 " << "\n";
        std::cout << " q) 退出\n";
        std::cout << "> " << std::flush;

        std::string input;
        if (!std::getline(std::cin, input)) break;
        if (input.empty()) continue;

        if (input == "q" || input == "Q") {
            std::cout << "退出程序。\n";
            break;
        }

        int idx = 0;
        try {
            idx = std::stoi(input);
        } catch (...) {
            std::cout << "请输入正确编号。\n";
            continue;
        } 

        try {
            if (idx == 1) {
                if (motor_action == "启动") {
                    // motor.start();
                    motor_action = "停止";
                } else {
                    // motor.stop();
                    motor_action = "启动";
                }
            } else if (idx == 2) {
                if (imu_action == "启动") {
                    // imu.start();
                    imu_action = "停止";
                } else {
                    // imu.stop();
                    imu_action = "启动";
                }
            } else {
                std::cout << "无效操作编号。\n";
                continue;
            }
        }catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    return 0;
}