#include "imu_modbus_rtu.h"
#include "log.h"

extern EventBus modbus_bus_;

void IMUModbusRTU::imu_start() {
    modbus_bus_.subscribe<ModbusData>("to_imu", [this](const ModbusData& data) {
        AINFO<<"回调更新imu数据";
        imu_update_data(data);
    });
}

void IMUModbusRTU::imu_update_data(const ModbusData& data) {
    std::lock_guard<std::mutex> lock(data_mutex);
    AINFO<<"开始更新IMU数据";
    // 更新姿态数据
    current_data.roll = data.rollAngle;
    current_data.pitch = data.pitchAngle;
    current_data.rudder = data.rudderAngle;
    
    // 更新速度数据
    current_data.speed = data.speed;
    
    // 更新位置数据
    current_data.longitude = data.longitude;
    current_data.latitude = data.latitude;
    
    // 更新主机数据
    current_data.left_rpm = data.leftEngineSpeed;
    current_data.right_rpm = data.rightEngineSpeed;
    current_data.left_gear = data.leftEngineGear;
    current_data.right_gear = data.rightEngineGear;
    
    // 更新时间戳
    current_data.timestamp = static_cast<uint32_t>(data.timestamp);
    
    // 其他默认值
    current_data.rpm = (data.leftEngineSpeed + data.rightEngineSpeed) / 2.0f; // 平均转速
    current_data.heading = 0.0f; // 默认值，可根据实际情况调整
    current_data.yaw = 0.0f; // 默认值，可根据实际情况调整
    current_data.gps_time = ""; // 默认值，可根据实际情况调整
    AINFO<<"data数据：controlMode="<<data.controlMode<<", manualLeftExtend="<<data.manualLeftExtend<<", manualRightExtend="<<data.manualRightExtend<<", autoModeParam="<<data.autoModeParam<<", rollAngle="<<data.rollAngle<<", pitchAngle="<<data.pitchAngle<<", rudderAngle="<<data.rudderAngle<<", speed="<<data.speed<<", timestamp="<<data.timestamp<<", longitude="<<data.longitude<<", latitude="<<data.latitude<<", leftEngineSpeed="<<data.leftEngineSpeed<<", rightEngineSpeed="<<data.rightEngineSpeed<<", leftEngineGear="<<data.leftEngineGear<<", rightEngineGear="<<data.rightEngineGear;
    AINFO<<"IMU数据更新完成：roll="<<current_data.roll<<", pitch="<<current_data.pitch<<", rudder="<<current_data.rudder<<", speed="<<current_data.speed<<", timestamp="<<current_data.timestamp<<", longitude="<<current_data.longitude<<", latitude="<<current_data.latitude<<", left_rpm="<<current_data.left_rpm<<", right_rpm="<<current_data.right_rpm<<", left_gear="<<current_data.left_gear<<", right_gear="<<current_data.right_gear<<", rpm="<<current_data.rpm<<", heading="<<current_data.heading<<", yaw="<<current_data.yaw;
}

ImuData IMUModbusRTU::getData() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}
