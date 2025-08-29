#ifndef LINUXPC_H
#define LINUXPC_H

#include <vector>
#include <string>

class LinuxPc {
public:
    LinuxPc();
    
    // 获取CPU温度（摄氏度）
    float getCPUTemperature();
    
    // 获取CPU核心电压（伏特）
    double getCPUVotage();
    
    // 获取内存使用率（百分比）
    float getMemoryUsage();
    
    // 获取磁盘使用率（百分比）
    float getDiskUsage(const std::string& path = "/");
    
    // 获取CPU使用率（百分比）
    float getCPUUsage();

private:
    // 读取文件内容到字符串
    std::string readFile(const std::string& path);
    
    // CPU使用率计算的历史数据
    struct CPUData {
        unsigned long long user;
        unsigned long long nice;
        unsigned long long system;
        unsigned long long idle;
    };
    CPUData lastCPUData;
    bool firstCPURun = true;
};

#endif