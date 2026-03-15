#pragma once
#include "data_struct.h"
#include <mutex>

class IMUModbusRTU {
public:
    void imu_start();
    void imu_update_data(const ModbusData& data);
    ImuData getData();
private:
    // 数据存储和线程安全
    ImuData current_data;
    std::mutex data_mutex;
};
