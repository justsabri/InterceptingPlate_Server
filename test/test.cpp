#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include "motor.h"
#include "log.h"
#include <nlohmann/json.hpp>

// 电机数据结构
struct Motor {
    int id;
    double position;   // 当前角度
    double offset;     // 零点偏移
};

bool is_float(const std::string& s, double& value) {
    try {
        size_t idx;
        value = std::stod(s, &idx);
        // idx 表示成功解析到的位置，如果没解析到末尾说明后面有非法字符
        if (idx != s.size()) {
            return false;
        }
        return true;
    } catch (const std::invalid_argument&) {
        return false; // 不能转换
    } catch (const std::out_of_range&) {
        return false; // 超出 double 范围
    }
}

void createMotorInstance(std::vector<Motor>& vec) {
    // 读取配置
    namespace fs = std::filesystem;

    std::string config_path = "config/config.json";
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
        vec.push_back(Motor{std::stoi(motor_id,nullptr,16), 0, 0}); 
    }
}

void show_motor_menu(Motor& motor) {
    MotorParser::getInstance().setPositionModeAndTarget(0, motor.id);
    while (true) {
        // 显示电机状态
        std::cout << "\n===== 电机 " << motor.id << " =====\n";
        motor.position = MotorParser::getInstance().getMotorPosition(motor.id);
        motor.offset = MotorParser::getInstance().getPositionOffset(motor.id);
        double max = MotorParser::getInstance().getMaxForwardPosition(motor.id);
        double min = MotorParser::getInstance().getMinReservePosition(motor.id);
        double voltage = MotorParser::getInstance().getEncoderBatteryVoltage(motor.id)*0.01;
        std::cout << "\n===== 电机 " << motor.id << " =====\n";
        std::cout << "当前位置: " << motor.position << "°\n";
        std::cout << "偏移位置: " << std::fixed << std::setprecision(0) << motor.offset << "\n";
        std::cout << "最大允许位置: " << max << "°\n";
        std::cout << "最小允许位置: " << min << "°\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "编码器电池电压: " << voltage << " v\n";
        std::cout << "----------------------\n";
        std::cout << "1) 左转 1°\n";
        std::cout << "2) 右转 1°\n";
        std::cout << "3) 左转 x°\n";
        std::cout << "4) 右转 x°\n";
        std::cout << "5) 转到 x°【" << min <<"-" << max << "°】\n";
        std::cout << "6) 设置当前位置为零点\n";
        std::cout << "7) 设置最大允许位置：（角度）\n";
        std::cout << "8) 设置最小允许位置：（角度）\n";
        std::cout << "9) 刷新电机状态\n";
        std::cout << "b) 返回上一级\n";
        std::cout << "> " << std::flush;

        std::string input;
        if (!std::getline(std::cin, input)) break;
        if (input.empty()) continue;

        if (input == "b" || input == "B") {
            break; // 返回上一级
        } else if (input == "1") {
            MotorParser::getInstance().setPositionModeAndTarget(motor.position-1, motor.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else if (input == "2") {
            MotorParser::getInstance().setPositionModeAndTarget(motor.position+1, motor.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else if (input == "3") {
            std::cout << "请输入需要左转的角度：（正数）\n";
            std::string s;
            std::getline(std::cin, s);
            double theta = -1;
            if (!is_float(s, theta) || theta < 0) {
                std::cout << "输入数据不合法，请重新选择后输入纯数字（正数）\n";
                continue;
            }
            motor.position -= theta;
            MotorParser::getInstance().setPositionModeAndTarget(motor.position, motor.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else if (input == "4") {
            std::cout << "请输入需要右转的角度：（正数）\n";
            std::string s;
            std::getline(std::cin, s);
            double theta = -1;
            if (!is_float(s, theta) || theta < 0) {
                std::cout << "输入数据不合法，请重新选择后输入纯数字（正数）\n";
                continue;
            }
            motor.position += theta;
            MotorParser::getInstance().setPositionModeAndTarget(motor.position, motor.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));    
        } else if (input == "5") {
            std::cout << "请输入目标角度：【" << min <<"-" << max << "°】\n";
            std::string s;
            std::getline(std::cin, s);
            double theta = -1;
            if (!is_float(s, theta) || theta < min || theta > max) {
                std::cout << "输入数据不合法，请重新选择后输入纯数字（正数）\n";
                continue;
            }
            motor.position = theta;
            MotorParser::getInstance().setPositionModeAndTarget(motor.position, motor.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else if (input == "6") {
            int32_t target_offset = motor.position/360*121*65535 + motor.offset;
            MotorParser::getInstance().setPositionOffset(target_offset, motor.id);
            MotorParser::getInstance().flush(motor.id);
            // motor.position = 0;
            std::cout << "零点已设置。\n";
        } else if (input == "7") {
            std::cout << "请输入目标角度：\n";
            std::string s;
            std::getline(std::cin, s);
            double theta = -1;
            if (!is_float(s, theta)) {
                std::cout << "输入数据不合法，请重新选择后输入纯数字\n";
                continue;
            }
            MotorParser::getInstance().setMaxForwardPosition(theta, motor.id);
            MotorParser::getInstance().flush(motor.id);    
        } else if (input == "8") {
            std::cout << "请输入目标角度：\n";
            std::string s;
            std::getline(std::cin, s);
            double theta = -1;
            if (!is_float(s, theta)) {
                std::cout << "输入数据不合法，请重新选择后输入纯数字\n";
                continue;
            }
            MotorParser::getInstance().setMinReversePosition(theta, motor.id);
            MotorParser::getInstance().flush(motor.id);
        } else if (input == "9") {
            std::cout << "重新获取电机状态...\n";
            continue;     
        } else {
            std::cout << "无效输入。\n";
        }
    }
}

int main() {
    InitLog("Test");

    // 初始化 4 个电机
    int res = MotorParser::getInstance().init("can0", true);
    if (res != 0) {
        std::string err;
        switch (res)
        {
            case 1:
                err = "创建套接子失败";
                break;
            case 2:
                err = "获取CAN接口索引失败";
                break;
            case 3:
                err = "绑定CAN套接字失败";
                break;
            default:
                err = "未知错误";
                break;
        }
        std::cout << "初始化can通信失败，错误原因：" << err << "\n";
        std::cout << "按任意键+回车退出..." << std::endl;
        std::cin.get(); // 等待输入
        return 1;
    }

    MotorParser::getInstance().stopReceiveThread();

    std::vector<Motor> motors;
    createMotorInstance(motors);

    std::map<int, int> index2id;
    std::cout << "\n----------------------------------------------------------\n";
    while (true) {
        std::cout << "\n===== 主菜单 =====\n";
        for (int i = 0; i < motors.size(); ++i) {
            std::cout << (i + 1) << ") 电机 " << motors[i].id << "\n";
            index2id.insert({i+1, motors[i].id});
        }
        std::cout << "q) 退出\n";
        std::cout << "> " << std::flush;

        std::string input;
        if (!std::getline(std::cin, input)) break;
        if (input.empty()) continue;

        if (input == "q" || input == "Q") {
            std::cout << "退出程序。\n";
            break;
        }

        try {
            int idx = std::stoi(input);
            if (index2id.find(idx) != index2id.end()) {
                std::cout << "进入电机" << motors[idx - 1].id << std::endl;
                show_motor_menu(motors[idx - 1]);
            } else {
                std::cout << "无效电机编号。\n";
            }
        } catch (...) {
            std::cout << "请输入正确编号。\n";
        }
    }

    return 0;
}
