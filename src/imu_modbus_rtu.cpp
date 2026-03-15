#include "imu_modbus_rtu.h"

extern EventBus modbus_bus_;

void IMUModbusRTU::imu_start() {
    modbus_bus_.subscribe<ModbusData>("to_imu", [this](const ModbusData& data) {
        imu_update_data(data);
    });
}

void IMUModbusRTU::imu_update_data(const ModbusData& data) {
    // data值更新到current_data中
}

ImuData IMUModbusRTU::getData() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}
