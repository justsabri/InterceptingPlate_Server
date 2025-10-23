#include "virtual_devices.h"
#include <iostream>
#include <filesystem>

bool canInterfaceExists(const std::string& ifname) {
    int sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ-1);
    ifr.ifr_name[IFNAMSIZ-1] = '\0';

    bool exists = (ioctl(sock, SIOCGIFINDEX, &ifr) != -1);
    close(sock);
    return exists;
}

bool fileExists(const char* path) {
    struct stat buffer;
    return (stat(path, &buffer) == 0);
}

void createVirtualSerialPort() {
    if (!fileExists("/tmp/ttyV0")) {
        std::cout << "Creating virtual serial ports..." << std::endl;
        system("socat -d -d pty,link=/tmp/ttyV0,raw,echo=0 pty,link=/tmp/ttyV1,raw,echo=0 &");
        sleep(1);
    } else {
        std::cout << "Virtual ports already exist." << std::endl;
    }
}

void createMotorInstance(std::vector<std::unique_ptr<VirtualMotor>>& vec) {
    // 读取配置
    namespace fs = std::filesystem;

    std::string config_path = "../config/config.json";
    std::cout << "JSON" << config_path << std::endl;
    
    if (!fs::exists(config_path)) {
        std::cout << "Config file does not exist: " << config_path << std::endl;
        return;
    }

    std::ifstream config_file(config_path);
    if (!config_file) {
        std::cout << "JSON fail" << config_path << std::endl;
        return;
    }
    nlohmann::json config = nlohmann::json::parse(config_file);

    for (const auto& [motor_id, motor_info] : config["motors"].items()) {
        std::string json_path = std::string("../test/virtual_motor_") + std::to_string(std::stoi(motor_id,nullptr,16)) + ".json";
        std::cout << "JSON path" << json_path << std::endl;
        vec.emplace_back(std::make_unique<VirtualMotor>("can0", json_path)); 
    }
}

int main() {
    if (!canInterfaceExists("can0")) {
        std::cout << "create can0" << std::endl;
        std::string scriptPath = "../test/setup_vcan.sh";
        int ret = system(scriptPath.c_str());
        if (ret != 0) {
            std::cerr << "Failed to execute script: " << scriptPath << " (return code " << ret << ")\n";
            return false;
        }

        // 给内核一点时间创建接口
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    createVirtualSerialPort();
    // InitLog("Test");
    std::vector<std::unique_ptr<VirtualMotor>> motors;
    createMotorInstance(motors);
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
                    std::cout << "电机启动\n";
                    for (auto& motor : motors) {
                        motor->start();
                    }
                    motor_action = "停止";
                } else {
                    for (auto& motor : motors) {
                        motor->stop();
                    }
                    motor_action = "启动";
                }
            } else if (idx == 2) {
                if (imu_action == "启动") {
                    imu.start();
                    imu_action = "停止";
                } else {
                    imu.stop();
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