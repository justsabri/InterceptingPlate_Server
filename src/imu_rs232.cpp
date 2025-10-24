#include "imu_rs232.h"
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <log.h>

IMURS232::IMURS232() {

}
//返回值：0：正常开启   1：打开设备失败  2：获取当前配置失败  3： 应用设置失败
int IMURS232::init(const std::string& device, int baud) {
    // 打开串口设备（非阻塞模式）
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0) {
        AERROR << "open failed: " << device <<std::endl;
        return 1;
    }

    // 获取当前串口配置
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        AERROR << "tcgetattr failed"<<std::endl;
        return 2;
    }

    // 基础配置 [2,4](@ref)
    tty.c_cflag &= ~PARENB;        // 无校验位
    tty.c_cflag &= ~CSTOPB;         // 1位停止位
    tty.c_cflag &= ~CSIZE;          // 清除数据位掩码
    tty.c_cflag |= CS8;             // 8位数据位
    tty.c_cflag &= ~CRTSCTS;        // 禁用硬件流控
    tty.c_cflag |= CREAD | CLOCAL;  // 启用接收，忽略调制解调器信号

    // 输入模式配置 [9](@ref)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL);

    // 输出模式配置
    tty.c_oflag &= ~OPOST;  // 原始输出模式

    // 本地模式配置
    tty.c_lflag &= ~ICANON;  // 非规范模式（原始数据）
    tty.c_lflag &= ~ECHO;    // 禁用回显
    tty.c_lflag &= ~ISIG;    // 禁用信号字符

    // 超时设置（100ms超时）[9](@ref)
    tty.c_cc[VTIME] = 1;   // 0.1秒超时
    tty.c_cc[VMIN] = 0;    // 最小读取0字节

    // 设置波特率
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 应用配置
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        AERROR<<"tcsetattr failed"<<std::endl;
        return 3;
    }

    // 清空缓冲区
    tcflush(fd_, TCIOFLUSH);

    // 初始化看门狗
    rs232_heartbeat_ = std::make_unique<Heartbeat>(500, 3000);
    feed_count = 0;
    rs232_heartbeat_->setTimeoutCallback([this](){
        heartbeatTimeout();
    });
    rs232_heartbeat_->setRecoverCallback([this](){
        heartbeatRecover();
    });
    rs232_heartbeat_->start();

    return 0;
}
//返回值 0:读取到完整数据  1：读取失败  2:未读取到完整数据包
int IMURS232::readData() {
    header_found = false;
    packet.clear();
    ssize_t bytes_read = 0;
    for (int i = 0; i < PACKET_SIZE; i++) {
        bytes_read = read(fd_, buffer, 1);
        if (bytes_read < 0) {
            // std::cerr << "Error: 读取串口失败" << std::endl;
            return 1;
        }
        if (buffer[0] == HEADER_BYTE1) {
            packet.push_back(buffer[0]);
            bytes_read = read(fd_, buffer, 1);
            if (bytes_read < 0) {
                // std::cerr << "Error: 读取串口失败" << std::endl;
                return 1;
            }
            if (buffer[0] == HEADER_BYTE2) {
                packet.push_back(buffer[0]);
                header_found = true;
                break;
            }
        }
    }
    AINFO << "virtual " << bytes_read << " " << header_found;
    if(!header_found)
        return 2;
    
    bytes_read = read(fd_, buffer, PACKET_SIZE - 2);
    if (bytes_read < 0) {
        // std::cerr << "Error: 读取串口失败" << std::endl;
        return 1;
    }
    packet.insert(packet.end(), buffer, buffer + sizeof(buffer)/sizeof(buffer[0]));
    parserData(packet.data());

    // 喂狗
    AINFO << "IMU feed" << feed_count;
    if (feed_count++ == 10) {
        feed_count = 0;
        rs232_heartbeat_->feed();
    }

    return 0;
}
//获取解析后的数据
ImuData IMURS232::getdata(){
    return imu_data;
}
void IMURS232::printData(){
    AINFO << "====raw data====";  //打印原始数据
    for (size_t i = 0; i < packet.size(); ++i) {
        // 两位16进制输出，不足补0
        AINFO << std::hex << std::setw(2) << std::setfill('0') 
                << static_cast<int>(packet[i]) << " ";
    }
    AINFO << std::endl;

    AINFO << "====imu data===="<<std::endl;   //打印解析后的数据
    // 1. IMU状态数据
    AINFO << "=== IMU状态 ===" << std::endl;
    AINFO << "imu_status_key: " << static_cast<int>(imu_data.imu_status_key) << std::endl;
    AINFO << "imu_work_status: " << static_cast<int>(imu_data.imu_work_status) << std::endl;
    AINFO << std::endl;

    // 2. GPS数据
    AINFO << "=== GPS数据 ===" << std::endl;
    AINFO << "gps_week: " << imu_data.gps_week << std::endl;
    AINFO << "gps_millisecond: " << imu_data.gps_millisecond << std::endl;
    AINFO << "GNSS_staus: " << static_cast<int>(imu_data.GNSS_staus) << std::endl;
    AINFO << std::endl;

    // 3. 姿态数据
    AINFO << "=== 姿态数据 ===" << std::endl;
    AINFO << "yaw: " << imu_data.yaw << std::endl;
    AINFO << "pitch: " << imu_data.pitch << std::endl;
    AINFO << "roll: " << imu_data.roll << std::endl;
    AINFO << "satellite_num: " << static_cast<int>(imu_data.satellite_num) << std::endl;
    AINFO << "posture_status: " << static_cast<int>(imu_data.posture_status) << std::endl;
    AINFO << std::endl;

    // 4. 位置数据
    AINFO << "=== 位置数据 ===" << std::endl;
    AINFO << "latitude: " << imu_data.latitude << std::endl;
    AINFO << "longitude: " << imu_data.longitude << std::endl;
    AINFO << std::endl;

    // 5. 速度数据
    AINFO << "=== 速度数据 ===" << std::endl;
    AINFO << "altitude: " << imu_data.altitude << std::endl;
    AINFO << "north_velocity: " << imu_data.north_velocity << std::endl;
    AINFO << "east_velocity: " << imu_data.east_velocity << std::endl;
    AINFO << "down_velocity: " << imu_data.down_velocity << std::endl;
    AINFO << std::endl;

    // 6. 陀螺数据
    AINFO << "=== 陀螺数据 ===" << std::endl;
    AINFO << "gyro_x: " << imu_data.gyro_x << std::endl;
    AINFO << "gyro_y: " << imu_data.gyro_y << std::endl;
    AINFO << "gyro_z: " << imu_data.gyro_z << std::endl;
    AINFO << "heading: " << imu_data.heading << std::endl;
    AINFO << std::endl;

    // 7. 加速度与温度
    AINFO << "=== 加速度与温度 ===" << std::endl;
    AINFO << "acc_x: " << imu_data.acc_x << std::endl;
    AINFO << "acc_y: " << imu_data.acc_y << std::endl;
    AINFO << "acc_z: " << imu_data.acc_z << std::endl;
    AINFO << "temperature: " << imu_data.temperature << std::endl;
    AINFO << "antenna_type: " << static_cast<int>(imu_data.antenna_type) << std::endl;
}

void IMURS232::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

IMURS232::~IMURS232() {
    close();
}

uint8_t IMURS232::parseUint8(const uint8_t *data){
    uint8_t value = 0;
    return *data;
}
uint16_t IMURS232::parseUint16(const uint8_t *data){
    uint16_t value = 0;
    value |= static_cast<uint8_t>(data[0]) << 0;  // 低位字节
    value |= static_cast<uint8_t>(data[1]) << 8;
    return value;
}
int16_t IMURS232::parseInt16(const uint8_t *data){
    int16_t value = 0;
    value |= static_cast<uint8_t>(data[0]) << 0;  // 低位字节
    value |= static_cast<uint8_t>(data[1]) << 8;
    return value;
}
uint32_t IMURS232::parseUint32(const uint8_t *data) {
    uint32_t value = 0;
    value |= static_cast<uint8_t>(data[0]) << 0;  // 低位字节
    value |= static_cast<uint8_t>(data[1]) << 8;
    value |= static_cast<uint8_t>(data[2]) << 16;
    value |= static_cast<uint8_t>(data[3]) << 24; // 高位字节
    return value;
}
int32_t IMURS232::parseInt32(const uint8_t *data) {
    int32_t value = 0;
    value |= static_cast<uint8_t>(data[0]) << 0;  // 低位字节
    value |= static_cast<uint8_t>(data[1]) << 8;
    value |= static_cast<uint8_t>(data[2]) << 16;
    value |= static_cast<uint8_t>(data[3]) << 24; // 高位字节
    return value;
}

void IMURS232::parserData(uint8_t *data){  //参照pdf设置缩放值
    imu_data.gps_millisecond = parseUint32(data + 6);
    imu_data.gps_week = parseUint16(data + 10);
    imu_data.latitude = parseInt32(data + 12)*(1e-7);
    imu_data.longitude = parseInt32(data + 16) * (1e-7);
    imu_data.altitude = parseInt32(data + 20);
    imu_data.north_velocity = parseInt16(data + 24)* 0.01;
    imu_data.east_velocity = parseInt16(data + 26) * 0.01;
    imu_data.down_velocity = parseInt16(data + 28) * 0.01;

    //------------------------------------------------------------
    //---原始水平惯导布放方式------
    imu_data.roll = parseInt16(data + 30) * 0.01;
    imu_data.pitch = parseInt16(data + 32)* 0.01;
    imu_data.yaw = parseUint16(data + 34)  * 0.01;
    AERROR<< "------------------------------------------------";
    // 3. 姿态数据
    AERROR << "=== 姿态数据 ===" ;
    AERROR << "yaw: " << imu_data.yaw ;
    AERROR << "pitch: " << imu_data.pitch ;
    AERROR << "roll: " << imu_data.roll ;
    //---新的垂直惯导布放方式------
    // imu_data.roll += 180.0;
    AERROR << "roll 2: " << imu_data.roll;
    // 横倾角--纵倾角  反向 20250807
    double new_pitch = -imu_data.roll;
    double new_roll = -imu_data.pitch;
    if (new_pitch < -90.0) {
        new_pitch += 360;
    }
    imu_data.pitch = new_pitch;
    imu_data.roll = new_roll;
    AERROR << "=== 姿态数据 安装位置转换后===";
    AERROR << "yaw: " << imu_data.yaw;
    AERROR << "pitch: " << imu_data.pitch;
    AERROR << "roll: " << imu_data.roll;
    //------------------------------------------------------------

    imu_data.acc_x = parseInt16(data + 36) * 0.001;
    imu_data.acc_y = parseInt16(data + 38) * 0.001;
    imu_data.acc_z = parseInt16(data + 40) * 0.001;
    imu_data.gyro_x = parseInt16(data + 42) * 0.01;
    imu_data.gyro_y = parseInt16(data + 44) * 0.01;
    imu_data.gyro_z = parseInt16(data + 46) * 0.01;
    imu_data.heading = parseInt16(data + 48)* 0.01;
    imu_data.antenna_type = parseUint8(data + 50);
    imu_data.temperature = parseUint8(data + 51) - 80;
    imu_data.satellite_num = parseUint8(data + 52);
    imu_data.GNSS_staus = parseUint8(data + 53);
    imu_data.posture_status = parseUint8(data + 54);
    imu_data.imu_status_key = parseUint8(data + 55);
    imu_data.imu_work_status = parseUint8(data + 56);
    AINFO << "parser imu: " << imu_data.disconnect;
    uint32_t crc32 = parseUint32(data + 57);
}

void IMURS232::heartbeatTimeout() {
    AINFO << "feed timeout";
    imu_data.disconnect = true;
}

void IMURS232::heartbeatRecover() {
    AINFO << "feed recover";
    imu_data.disconnect = false;
}
