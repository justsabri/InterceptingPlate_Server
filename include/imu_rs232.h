#pragma once
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip> 
#include <vector>
#include "data_struct.h"
#include "HeartBeat.h"
//RS232-1    /dev/ttyS8
//RS232-2    /dev/ttyS9
//RS232-3    /dev/ttyS7
//RS232-4    /dev/ttyS4
//RS232-5    /dev/ttyS3

class IMURS232 {
public:
    // 构造函数（指定串口设备和波特率）
    explicit IMURS232();
    
    // 打开并配置串口 返回值：0：正常开启   1：打开设备失败  2：获取当前配置失败  3： 应用设置失败
    int init(const std::string& device, int baud);
    
    // 读取串口数据
    int readData();
    void printData();    //显示数据测试用

    //获取数据
    ImuData getdata();
    // 关闭串口
    void close();
    
    // 析构函数（自动关闭）
    ~IMURS232();

private:
    ImuData imu_data;
    static const uint8_t HEADER_BYTE1 = 0xAA;
    static const uint8_t HEADER_BYTE2 = 0x55;
    static const size_t PACKET_SIZE = 51; // 包含帧头的完整数据包长度
    std::vector<uint8_t> packet;    //数据包
    bool header_found = false;  //帧头获取标志
    uint8_t buffer[PACKET_SIZE];  // 单字节缓冲区
    int fd_ = -1;         // 文件描述符
    std::unique_ptr<Heartbeat> rs232_heartbeat_;
    int feed_count;

    //解析数据
    void parserData(uint8_t *data);
    //解析辅助函数
    uint8_t parseUint8(const uint8_t *data);
    uint16_t parseUint16(const uint8_t *data);
    int16_t parseInt16(const uint8_t *data);
    int32_t parseInt32(const uint8_t *data);
    uint32_t parseUint32(const uint8_t *data);
    float parseFloat(const uint8_t *data);
    double parseDouble(const uint8_t *data);
    void heartbeatTimeout();
    void heartbeatRecover();
};