#ifndef PROJECT_DATA_CONTAINER_H_
#define PROJECT_DATA_CONTAINER_H_

#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <functional>
#include "motor.h"
#include "imu_rs232.h"
#include "data_struct.h"
#include "pc.h"
// 数据容器类：用于集中存储和管理应用数据
class DataContainer {
public:
    //获取硬件模块数据刷新私有成员数据  调用一次刷新一次数据
    int refreshMotorData(void);
    int refreshImuData(void);
    void refreshPcData(void);
    //返回值:motor and imu InitdeviceStatus struct
    InitDeviceStatus initDevice(void);
private:
    IMURS232   imu;
    LinuxPc    pc;

        // 线程控制相关变量
    std::thread motor_thread_;
    std::thread imu_thread_;
    std::thread pc_thread_;
    std::atomic<bool> running_{false};
    int motor_freq_hz_ = 10;  // 默认频率10Hz
    int imu_freq_hz_ = 20;    // 默认频率20Hz
    int pc_freq_hz_ = 1;      // 默认频率1Hz

    typedef struct {
        int motor_num;
        std::vector<int> motor;
    } Motor_Config;

    Motor_Config motor_config_;
public:
    //电机数据成员
    std::map<int, MotorData> motor_data_;

    //惯导数据成员
    ImuData imu_data_;

    //pc datastruce member
    LinuxPcData linux_pc_data_;

    InitDeviceStatus init_device_status_;

    void motor_thread();
    void imu_thread();
    void pc_thread();
    void init_threads(int motor_freq, int imu_freq, int pc_freq);
    void stop_threads();

    using MotorDataCallback = std::function<void(const std::map<int, MotorData>&)>;
    using ImuDataCallback = std::function<void(const ImuData&)>;
    using PcDataCallback = std::function<void(const LinuxPcData&)>;

    void motorData(MotorDataCallback cb);
    void imuData(ImuDataCallback cb);
    void pcData(PcDataCallback cb);


    MotorDataCallback callback_motor;
    ImuDataCallback callback_imu;
    PcDataCallback callback_pc;
    std::mutex motor_mutex_;
    std::mutex imu_mutex_;
    std::mutex pc_mutex_;
};

#endif  // PROJECT_DATA_CONTAINER_H_