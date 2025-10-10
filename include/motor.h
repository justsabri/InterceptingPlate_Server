#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cmath>
#include <vector>
#include <endian.h>
#include <thread>
#include <atomic>
#include <iomanip>
#include <mutex>
#include "data_struct.h"
#include "HeartBeat.h"

typedef enum {
    PARAM_TYPE_INT32,
    PARAM_TYPE_DOUBLE,
    PARAM_TYPE_STRING,
    PARAM_TYPE_THETA,
    PARAM_TYPE_CODERTHETA,
    PARAM_TYPE_ERROR
} MotorParamType;

struct MotorParamItem {
    std::string name;
    MotorParamType type;
    size_t offset;
    uint16_t can_cmd;
};

// 传感器数据结构体
class MotorParser {
private:
    int socket_fd;
    double gearRatio;
    std::mutex mtx_;
    std::map<int, MotorData> motor_data_;
    std::thread read_thread_;
    std::atomic<bool> running_{ true };
    uint32_t wait_time_; // 等待数据更新时间 ms

    // heartbeat
    std::map<int, std::unique_ptr<Heartbeat>> motor_heartbeat_;
    
    // 私有构造函数防止外部实例化
    MotorParser();
    MotorParser(const MotorParser&) = delete;
    MotorParser& operator=(const MotorParser&) = delete;
public:
    // 公共接口获取单例
    static MotorParser& getInstance();

    ~MotorParser();
    //返回值 0:初始化完成   1：创建套接子失败   2：获取CAN接口索引失败   3：绑定CAN套接字失败
    int init(const std::string& can_channel, bool is_test=false);
    void send(int can_id, uint8_t cmd, const std::vector<uint8_t>& data);
    std::vector<uint8_t> receive(uint8_t cmd);

    int32_t parseInt32(const std::vector<uint8_t>& data);
    MotorErrorStatus getMotorErrorStatus(int can_id); //获取电机错误状态

    int32_t getMotorErrorCode(int can_id); //获取电机错误码
    std::string getMotorMode(int can_id);  // 获取电机运行模式
    double getMotorCurrent(int can_id);     // 获取电机电流
    double getMotorVoltage(int can_id);     // 获取电机（母线）电压
    double calculateMotorPower(double voltage, double current); //计算电机功率
    double getMotorSpeed(int can_id);   // 获取电机速度
    double getMotorPosition(int can_id);    // 获取电机当前位置
    double getMaxForwardAcceleration(int can_id);   // 获取最大正向加速度
    double getMinReverseAcceleration(int can_id);   // 获取最小负向加速度
    double getMaxForwardSpeed(int can_id);  // 获取最大正向允许速度
    double getMinReverseSpeed(int can_id);  // 获取最小负向允许速度
    double getMaxForwardPosition(int can_id);   // 获取电机最大正向位置
    double getMinReservePosition(int can_id);   // 获取电机最小负向位置
    double getMotorTemperature(int can_id); // 获取电机温度
    int32_t getPositionOffset(int can_id);  // 获取当前位置偏移
    double getEncoderBatteryVoltage(int can_id);    // 获取编码器电池电压

    void setPositionModeAndTarget(double targetAngle,int can_id);   // 设置位置模式、目标位置
    void setMaxForwardAcceleration(int32_t acceleration,int can_id);    // 设置电机最大正向加速度
    void setMinReverseAcceleration(int32_t acceleration,int can_id);    // 设置电机最小负向加速度
    void setMaxForwardSpeed(double speedDeg,int can_id);    // 设置最大正向允许速度
    void setMinReverseSpeed(double speedDeg,int can_id);    // 设置最小负向允许速度
    void setMaxForwardPosition(double angleDeg,int can_id); //设置最大正向位置
    void setMinReversePosition(double angleDeg,int can_id); // 设置最小负向位置
    void setPositionOffset(int32_t offset,int can_id);  // 设置位置偏移
    void setBaud(int32_t baud,int can_id);  //设置波特率  1000、500、250、125、100、50
    void flush(int can_id);
    //获取电机数据
    MotorData getMotorData(int can_id);     //获取电机数据

    void queryAll(int can_id);
    MotorParamItem* getItemByCanCmd(uint16_t can_cmd);
    void receiveLoop();
    void stopReceiveThread();
    void heartbeatTimeout(int can_id);
    void heartbeatRecover(int can_id);
};

#endif // MOTOR_H