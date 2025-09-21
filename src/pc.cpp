#include "pc.h"
#include <fstream>
#include <sstream>
#include <sys/statvfs.h>
#include <filesystem>
#include "log.h"
LinuxPc::LinuxPc() {
    // 初始化CPU数据
    lastCPUData = {0, 0, 0, 0};
}

std::string LinuxPc::readFile(const std::string& path) {
    std::ifstream file(path);
    return file ? std::string(std::istreambuf_iterator<char>(file), 
                             std::istreambuf_iterator<char>()) : "";
}

float LinuxPc::getCPUTemperature() {
    // 尝试不同温度传感器路径
    const std::vector<std::string> paths = {
        "/sys/class/thermal/thermal_zone0/temp",
    };

    for (const auto& path : paths) {
        std::string content = readFile(path);
        if (!content.empty()) {
            try {
                return std::stof(content) / 1000.0f; // 转换为摄氏度
            } catch (...) {
                continue;
            }
        }
    }
    return -273.15f; // 绝对零度表示错误
}

double LinuxPc::getCPUVotage() {
    // 电压通常从/sys文件系统获取
    const std::string path = "/sys/bus/iio/devices/iio:device0/in_voltage4_raw";
    std::ifstream file(path);
       // 读取原始值
    int raw_value;
    file >> raw_value;
    // AERROR << "===============原始电压："<<raw_value;
    if (raw_value)
    {
        double current_voltage = (static_cast<double>(raw_value) / 1024.0)*1.8*21.0;
        // AERROR <<"==============计算电压"<<current_voltage;
        return current_voltage;
    }
    
    return -1.0; // 错误值
}

float LinuxPc::getMemoryUsage() {
    std::ifstream file("/proc/meminfo");
    if (!file) return -1.0f;

    long total = -1, available = -1;
    std::string line;

    while (getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        long value;
        iss >> key >> value;
        
        if (key == "MemTotal:") total = value;
        else if (key == "MemAvailable:") available = value;
        
        if (total != -1 && available != -1) break;
    }

    if (total <= 0 || available < 0) return -1.0f;
    return 100.0f * (1.0f - static_cast<float>(available) / total);
}

float LinuxPc::getDiskUsage(const std::string& path) {
    struct statvfs stats;
    if (statvfs(path.c_str(), &stats) != 0) return -1.0f;
    
    const auto total = static_cast<float>(stats.f_blocks * stats.f_frsize);
    const auto free = static_cast<float>(stats.f_bfree * stats.f_frsize);
    
    return total > 0 ? 100.0f * (1.0f - free / total) : -1.0f;
}

float LinuxPc::getCPUUsage() {
    std::ifstream file("/proc/stat");
    if (!file) return -1.0f;

    std::string line;
    getline(file, line); // 读取第一行（总CPU数据）
    std::istringstream iss(line);
    
    std::string cpuLabel;
    unsigned long long user, nice, system, idle;
    iss >> cpuLabel >> user >> nice >> system >> idle;
    
    if (firstCPURun) {
        lastCPUData = {user, nice, system, idle};
        firstCPURun = false;
        return 0.0f;
    }
    
    // 计算两次采样的差值
    const unsigned long long totalDiff = 
        (user + nice + system + idle) - 
        (lastCPUData.user + lastCPUData.nice + lastCPUData.system + lastCPUData.idle);
    
    const unsigned long long idleDiff = idle - lastCPUData.idle;
    lastCPUData = {user, nice, system, idle};
    
    return totalDiff > 0 ? 100.0f * (1.0f - static_cast<float>(idleDiff) / totalDiff) : 0.0f;
}