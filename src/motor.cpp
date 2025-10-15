#include "motor.h"
#include <stdexcept>
#include <algorithm>
#include "log.h"
#include <sstream>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#define PARAM_PTR(data, p) ((void *)((uint8_t *)data + (p)->offset))

MotorParamItem g_param_table[] = {
    {"status", PARAM_TYPE_ERROR, offsetof(MotorData, status), 0x0A},                                    // 状态 获取频率1hz
    {"mode", PARAM_TYPE_STRING, offsetof(MotorData, mode), 0x03},                                       // 模式 获取频率1hz
    {"voltage", PARAM_TYPE_DOUBLE, offsetof(MotorData, voltage), 0x14},                                 // 电压 获取频率1hz
    {"current", PARAM_TYPE_DOUBLE, offsetof(MotorData, current), 0x04},                                 // 电机电流 获取频率10hz
    {"position", PARAM_TYPE_CODERTHETA, offsetof(MotorData, position), 0x08},                           // 电机位置 获取频率=电机频率
    {"position_offset", PARAM_TYPE_INT32, offsetof(MotorData, position_offset), 0x54},                  // 电机位置偏移
    {"encoder_battery_voltage", PARAM_TYPE_DOUBLE, offsetof(MotorData, encoder_battery_voltage), 0x78}, // 电机编码器电池电压  获取频率1hz
    {"max_forward_speed", PARAM_TYPE_THETA, offsetof(MotorData, max_forward_speed), 0x18},              // 最大前向速度  获取频率1hz
    {"min_reverse_speed", PARAM_TYPE_THETA, offsetof(MotorData, min_reverse_speed), 0x19},              // 最大后向速度  获取频率1hz
    {"max_forward_position", PARAM_TYPE_CODERTHETA, offsetof(MotorData, max_forward_position), 0x1A},   // 最大前向位置偏移  获取频率1hz
    {"min_reverse_position", PARAM_TYPE_CODERTHETA, offsetof(MotorData, min_reverse_position), 0x1B},   // 最大后向位置偏移  获取频率1hz
    {"temperature", PARAM_TYPE_DOUBLE, offsetof(MotorData, temperature), 0x31}};                        // 电机温度 获取频率1hz

// ====================== MotorParser单例实现 ======================
MotorParser &MotorParser::getInstance()
{
    static MotorParser instance; // 单例实例 (C++11线程安全)
    return instance;
}

MotorParser::MotorParser() : socket_fd(-1), gearRatio(121.0) {}

MotorParser::~MotorParser()
{

    stopReceiveThread();

    // 停止指令处理线程
    running_ = false;
    cmd_cv_.notify_all();
    if (command_thread_.joinable())
    {
        command_thread_.join();
    }

    if (socket_fd >= 0)
    {
        close(socket_fd);
        socket_fd = -1;
    }
}

// 返回值 0:初始化完成   1：创建套接子失败   2：获取CAN接口索引失败   3：绑定CAN套接字失败
int MotorParser::init(const std::string &can_channel, bool is_test)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    if (socket_fd >= 0)
    {
        return -1; // 防止重复初始化
    }

    // 1. 创建Socket
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0)
    {
        return 1;
    }

    // 2. 获取接口索引
    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_channel.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
    {
        close(socket_fd);
        return 2;
    }

    // 3. 绑定Socket到CAN接口
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(socket_fd);
        return 3;
    }

    // 4. 读取电机配置
    namespace fs = std::filesystem;
    fs::path config_path = fs::current_path() / "config/config.json";
    std::ifstream config_file(config_path);
    nlohmann::json config = nlohmann::json::parse(config_file);
    for (const auto &[motor_id, motor_info] : config["motors"].items())
    {
        motor_data_.insert({std::stoi(motor_id, nullptr, 16), MotorData()});
    }
    motor_freq_ = config["motor_freq"].get<int>();
    wait_time_ = 1000 / motor_freq_ * 0.5; // ms

    // 5. 启动数据接收线程
    running_ = !is_test;
    read_thread_ = std::thread(&MotorParser::receiveLoop, this);
    // 启动指令处理线程
    command_thread_ = std::thread(&MotorParser::commandProcessingThread, this);

    // 6.初始化看门狗
    for (const auto& [motor_id, motor_info] : config["motors"].items()) {
        int id = std::stoi(motor_id,nullptr,16);
        motor_heartbeat_.emplace(id, std::make_unique<Heartbeat>(500, 2000));
        motor_heartbeat_[id]->setTimeoutCallback([this, id](){
            heartbeatTimeout(id);
        });
        motor_heartbeat_[id]->setRecoverCallback([this, id](){
            heartbeatRecover(id);
        });
    }

    return 0;
}

// void MotorParser::send(int can_id, uint8_t cmd, const std::vector<uint8_t>& data) {

//     std::lock_guard<std::mutex> lock(mtx_);
//     if (socket_fd < 0) {
//         AERROR << "Socket not initialized";
//         return;
//     }
//     struct can_frame frame;
//     frame.can_id = can_id;

//     size_t total_len = std::min(data.size() + 1, static_cast<size_t>(8));
//     frame.can_dlc = static_cast<__u8>(total_len);

//     frame.data[0] = cmd;

//     // AERROR << "----------------MotorParser::send--------------------------";

//     for (size_t i = 0; i < total_len - 1; ++i) {
//         frame.data[i + 1] = data[i];
//     }

//     if (write(socket_fd, &frame, sizeof(frame)) == sizeof(frame)) {

//     }
// }

std::vector<uint8_t> MotorParser::receive(uint8_t cmd)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    if (socket_fd < 0)
    {
        AINFO << "=======MotorParser::socket_fd=======" << socket_fd;
        // return {};
    }
    struct can_frame frame;
    do
    {
        ssize_t nbytes = 0;
        // if (ioctl(socket_fd, FIONREAD, &nbytes) == 0) {
        //     if (nbytes == 0) {
        //         printf("缓冲区为空\n");
        //         return {};
        //     } else {
        //         printf("缓冲区有 %d 字节数据待处理\n", nbytes);
        //     }
        // }
        // AINFO << "before read";
        nbytes = read(socket_fd, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            AINFO << "=======MotorParser::nbytes=======" << nbytes;
            // return {};
        }

        std::ostringstream oss;
        // oss << "receive data=";
        for (int i = 0; i < frame.can_dlc; i++)
        {
            oss << " " << std::hex << static_cast<int>(frame.data[i]);
        }
        AINFO << oss.str();

        if (frame.data[0] == 0x0a)
        {
            break;
        }
    } while (frame.data[0] != cmd);

    // 故障
    if (frame.data[0] == 0x0a)
    {
        auto data = std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
        int32_t status = parseInt32(data);
        if (status & (1 << 0))
            AERROR << " - Internal software error\n";
        if (status & (1 << 1))
            AERROR << " - Overvoltage\n";
        if (status & (1 << 2))
            AERROR << " - Undervoltage\n";
        if (status & (1 << 4))
            AERROR << " - Startup error\n";
        if (status & (1 << 5))
            AERROR << " - Speed feedback error\n";
        if (status & (1 << 6))
            AERROR << " - Overcurrent\n";
        if (status & (1 << 7))
            AERROR << " - Other software error\n";
        if (status & (1 << 16))
            AERROR << " - Encoder communication error\n";
        if (status & (1 << 17))
            AERROR << " - Motor over temperature\n";
        if (status & (1 << 18))
            AERROR << " - Board over temperature\n";
        if (status & (1 << 19))
            AERROR << " - Driver chip error\n";

        AINFO << "=======MotorParser::frame.data===0x0a====" << status;
        // return {};

        return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
    }
    else
    {
        return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
    }
}

// ====================== 工具函数实现 ======================
int32_t MotorParser::parseInt32(const std::vector<uint8_t> &data)
{
    if (data.size() < 4)
        return 0;
    int32_t value =
        static_cast<int32_t>(data[1]) |
        (static_cast<int32_t>(data[2]) << 8) |
        (static_cast<int32_t>(data[3]) << 16) |
        (static_cast<int32_t>(data[4]) << 24);
    return le32toh(value);
}

MotorErrorStatus MotorParser::getMotorErrorStatus(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x0A, {}, PRIORITY_NORMAL);
    auto response = receive(0x0A);
    int32_t status = parseInt32(response);
    MotorErrorStatus err;
    err.bit0_softwareError = status & (1 << 0);
    err.bit1_overVoltage = status & (1 << 1);
    err.bit2_underVoltage = status & (1 << 2);
    err.bit4_startupError = status & (1 << 4);
    err.bit5_speedFeedbackError = status & (1 << 5);
    err.bit6_overCurrent = status & (1 << 6);
    err.bit7_softwareErrorOther = status & (1 << 7);
    err.bit16_encoderCommError = status & (1 << 16);
    err.bit17_motorOverTemp = status & (1 << 17);
    err.bit18_boardOverTemp = status & (1 << 18);
    err.bit19_driverChipError = status & (1 << 19);
    return err;
}

// //获取电机错误码
// int32_t MotorParser::getMotorErrorCode(int can_id){
//     send(can_id,0x0A, {});
//     auto response = receive();
//     int32_t status = parseInt32(response);
//     return status;
// }
// 获取电机运行模式
std::string MotorParser::getMotorMode(int can_id)
{

    sendMotorCommand(can_id, 0x03, {}, PRIORITY_NORMAL);
    auto response = receive(0x03);
    int32_t mode = parseInt32(response);
    switch (mode)
    {
    case -1:
        return "Fault mode";
    case 0:
        return "Stop mode";
    case 1:
        return "Current mode";
    case 2:
        return "Speed mode";
    case 3:
        return "Position mode";
    default:
        return "Unknown mode";
    }
}

// 获取电机电流
double MotorParser::getMotorCurrent(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x04, {}, PRIORITY_NORMAL);
    auto response = receive(0x04);
    return static_cast<double>(parseInt32(response));
}

// 获取电机（母线）电压
double MotorParser::getMotorVoltage(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x14, {}, PRIORITY_NORMAL);
    auto response = receive(0x14);
    return static_cast<double>(parseInt32(response));
}

// 计算电机功率
double MotorParser::calculateMotorPower(double voltage, double current)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    return voltage * current / 1000.0;
}

// 获取电机速度
double MotorParser::getMotorSpeed(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x06, {}, PRIORITY_NORMAL);
    auto response = receive(0x06);
    return (parseInt32(response) / 100.0 / gearRatio) * 360.0;
}

// 获取电机当前位置
double MotorParser::getMotorPosition(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x08, {}, PRIORITY_NORMAL);
    AINFO << "=======电机：获取电机当前位置=======，发送一字节0x08";

    //--------------------------------------------------------------------------------
    std::vector<uint8_t> response = receive(0x08);
    int len_res = response.size();
    AINFO << "=======电机：=======" << can_id << "=======接收当前位置原始can_frame字节=======" << len_res;
    if (response[0] != 0x08)
    {
        AERROR << "can帧错误!";
    }

    std::ostringstream oss;
    oss << "当前位置原始can_frame byte=";
    for (int i = 0; i < len_res; i++)
    {
        oss << " " << std::hex << static_cast<int>(response[i]);
    }
    AINFO << oss.str();

    // //--------------------------------------------------------------------------------
    // AINFO<<"=======第二次接收----------------------------------";
    // response = receive();
    // len_res = response.size();
    // AINFO<<"=======电机22222222222222222：======="<<can_id<<"=======接收当前位置原始can_frame字节======="<<len_res;
    // for (int i=0;i<len_res;i++)
    // {
    //     AINFO<<"=当前位置原始22222222222222222can_frame byte="<<response[i];
    // }

    // //--------------------------------------------------------------------------------
    // AERROR<<"=======第3次接收----------------------------------";
    // response = receive();
    // len_res = response.size();
    // AINFO<<"=======电机22222222222222222：======="<<can_id<<"=======接收当前位置原始can_frame字节======="<<len_res;
    // for (int i=0;i<len_res;i++)
    // {
    //     AINFO<<"=当前位置原始22222222222222222can_frame byte="<<response[i];
    // }

    //--------------------------------------------------------------------------------

    double value = (parseInt32(response) / 65536.0 / gearRatio) * 360.0;
    AINFO << "=======电机：=======" << can_id << "=======当前位置value=======" << value;

    return (parseInt32(response) / 65536.0 / gearRatio) * 360.0;
}

// 获取最大正向加速度
double MotorParser::getMaxForwardAcceleration(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x16, {}, PRIORITY_NORMAL);
    auto response = receive(0x16);
    return static_cast<double>(parseInt32(response));
}

// 获取最小负向加速度
double MotorParser::getMinReverseAcceleration(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x17, {}, PRIORITY_NORMAL);
    auto response = receive(0x17);
    return static_cast<double>(parseInt32(response));
}

// 获取最大正向允许速度
double MotorParser::getMaxForwardSpeed(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x18, {}, PRIORITY_NORMAL);
    auto response = receive(0x18);
    int32_t speed = parseInt32(response);
    return (speed / 100.0 / gearRatio) * 360.0;
}

// 获取最小负向允许速度
double MotorParser::getMinReverseSpeed(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x19, {}, PRIORITY_NORMAL);
    auto response = receive(0x19);
    int32_t speed = parseInt32(response);
    return (speed / 100.0 / gearRatio) * 360.0;
}

// 获取电机最大正向位置
double MotorParser::getMaxForwardPosition(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x1A, {}, PRIORITY_NORMAL);
    auto response = receive(0x1A);
    return (parseInt32(response) / 65536.0 / gearRatio) * 360.0;
}

// 获取电机最小负向位置
double MotorParser::getMinReservePosition(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x1B, {}, PRIORITY_NORMAL);
    auto response = receive(0x1B);
    return (parseInt32(response) / 65536.0 / gearRatio) * 360.0;
}

// 获取电机温度
double MotorParser::getMotorTemperature(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x31, {}, PRIORITY_NORMAL);
    auto response = receive(0x31);
    return static_cast<double>(parseInt32(response));
}

// 获取当前位置偏移
int32_t MotorParser::getPositionOffset(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x54, {}, PRIORITY_NORMAL);
    auto response = receive(0x54);
    return parseInt32(response);
}

// 获取编码器电池电压
double MotorParser::getEncoderBatteryVoltage(int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    sendMotorCommand(can_id, 0x78, {}, PRIORITY_NORMAL);
    auto response = receive(0x78);
    int32_t voltage = parseInt32(response);
    return static_cast<double>(voltage);
}

// 设置位置模式、目标位置
void MotorParser::setPositionModeAndTarget(double targetAngle, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    int32_t pos = static_cast<int32_t>((targetAngle / 360.0) * gearRatio * 65536.0);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(pos & 0xFF),
        static_cast<uint8_t>((pos >> 8) & 0xFF),
        static_cast<uint8_t>((pos >> 16) & 0xFF),
        static_cast<uint8_t>((pos >> 24) & 0xFF)};
    AINFO << targetAngle << " " << can_id;
    sendMotorCommand(can_id, 0x1E, data, PRIORITY_HIGH);
}

// 设置电机最大正向加速度
void MotorParser::setMaxForwardAcceleration(int32_t acceleration, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(acceleration & 0xFF),
        static_cast<uint8_t>((acceleration >> 8) & 0xFF),
        static_cast<uint8_t>((acceleration >> 16) & 0xFF),
        static_cast<uint8_t>((acceleration >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x22, data, PRIORITY_HIGH);
}

// 设置电机最小负向加速度
void MotorParser::setMinReverseAcceleration(int32_t acceleration, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(acceleration & 0xFF),
        static_cast<uint8_t>((acceleration >> 8) & 0xFF),
        static_cast<uint8_t>((acceleration >> 16) & 0xFF),
        static_cast<uint8_t>((acceleration >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x23, data, PRIORITY_HIGH);
}

// 设置最大正向允许速度
void MotorParser::setMaxForwardSpeed(double speedDeg, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    int32_t value = static_cast<int32_t>((speedDeg * gearRatio * 100) / 360.0);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x24, data, PRIORITY_HIGH);
}

// 设置最小负向允许速度
void MotorParser::setMinReverseSpeed(double speedDeg, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    int32_t value = static_cast<int32_t>((speedDeg * gearRatio * 100) / 360.0);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x25, data, PRIORITY_HIGH);
}

// 设置最大正向位置
void MotorParser::setMaxForwardPosition(double angleDeg, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    int32_t pos = static_cast<int32_t>((angleDeg / 360.0) * gearRatio * 65536.0);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(pos & 0xFF),
        static_cast<uint8_t>((pos >> 8) & 0xFF),
        static_cast<uint8_t>((pos >> 16) & 0xFF),
        static_cast<uint8_t>((pos >> 24) & 0xFF)};
    AINFO << "setMaxForwardPosition " << "canid " << can_id << angleDeg;
    sendMotorCommand(can_id, 0x26, data, PRIORITY_HIGH);
}

// 设置最小负向位置
void MotorParser::setMinReversePosition(double angleDeg, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    int32_t pos = static_cast<int32_t>((angleDeg / 360.0) * gearRatio * 65536.0);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(pos & 0xFF),
        static_cast<uint8_t>((pos >> 8) & 0xFF),
        static_cast<uint8_t>((pos >> 16) & 0xFF),
        static_cast<uint8_t>((pos >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x27, data, PRIORITY_HIGH);
}

// 设置位置偏移
void MotorParser::setPositionOffset(int32_t offset, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(offset & 0xFF),
        static_cast<uint8_t>((offset >> 8) & 0xFF),
        static_cast<uint8_t>((offset >> 16) & 0xFF),
        static_cast<uint8_t>((offset >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x53, data, PRIORITY_HIGH);
}

// 设置波特率  1000、500、250、125、100、50
void MotorParser::setBaud(int32_t baud, int can_id)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(baud & 0xFF),
        static_cast<uint8_t>((baud >> 8) & 0xFF),
        static_cast<uint8_t>((baud >> 16) & 0xFF),
        static_cast<uint8_t>((baud >> 24) & 0xFF)};
    sendMotorCommand(can_id, 0x3F, data, PRIORITY_HIGH);
}

void MotorParser::flush(int can_id)
{
    sendMotorCommand(can_id, 0x0E, {}, PRIORITY_NORMAL);
}

// 获取电机数据
//  MotorData MotorParser::getMotorData(int can_id){
//      // AINFO << "lock 1";
//      // std::lock_guard<std::mutex> lock(mtx_);
//      // AINFO <<"lock 2";
//      MotorData data;
//      data.mode = getMotorMode(can_id);
//      data.current = getMotorCurrent(can_id);
//      data.voltage = getMotorVoltage(can_id);
//      data.position = getMotorPosition(can_id);
//      AINFO<<"==========================电机："<<can_id<<"=================当前位置"<<data.position;
//      data.position_offset = getPositionOffset(can_id);

//     data.encoder_battery_voltage = getEncoderBatteryVoltage(can_id);
//     AINFO<<"==========================电机："<<can_id<<"=================电压 "<<data.encoder_battery_voltage;
//     data.max_forward_speed = getMaxForwardSpeed(can_id);
//     data.min_reverse_speed = getMinReverseSpeed(can_id);
//     data.max_forward_position = getMaxForwardPosition(can_id);
//     AINFO << "max_forward_position " << data.max_forward_position;
//     data.min_reverse_position = getMinReservePosition(can_id);
//     data.temperature = getMotorTemperature(can_id);
//     return data;
// }

MotorData MotorParser::getMotorData(int can_id)
{
    AINFO << "---------queryAll查询前------ " << "canid " << can_id;
    queryAll(can_id);
    AINFO << "---------queryAll查询后------ " << "canid " << can_id;
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_));
    return motor_data_[can_id];
}

void MotorParser::queryAll(int can_id)
{
    // 暂定实现
    // 位置获取频率20hz 电流获取频率10hz
    // 其他信息1hz
     int times = 0;
     if (times != motor_freq_)
     {
         MotorParamItem *item = &g_param_table[4]; // 电机位置 获取频率=motor_freq
         sendMotorCommand(can_id, item->can_cmd, {}, PRIORITY_NORMAL);
         times++;
     }
     else if (times % 2 == 0)
     {
         MotorParamItem *item = &g_param_table[3]; // 电机电流 获取频率=motor_freq/2
         sendMotorCommand(can_id, item->can_cmd, {}, PRIORITY_NORMAL);
     }
     else if (times == motor_freq_)
     {
         for (int i = 0; i < sizeof(g_param_table) / sizeof(MotorParamItem); i++)
         {
             MotorParamItem *item = &g_param_table[i];//所有数据 频率为1Hz
             sendMotorCommand(can_id, item->can_cmd, {}, PRIORITY_NORMAL);
         }
         times = 0;
     }

    // 改进实现-1
    // int times = 0;
    // if (times != 10)
    // {

    //     MotorParamItem *item = &g_param_table[4];
    //     sendMotorCommand(can_id, item->can_cmd, {}, PRIORITY_NORMAL);
    //     times++;
    // }
    // else
    // {
    //     for (int i = 0; i < sizeof(g_param_table) / sizeof(MotorParamItem); i++)
    //     {
    //         MotorParamItem *item = &g_param_table[i];
    //         sendMotorCommand(can_id, item->can_cmd, {}, PRIORITY_NORMAL);
    //     }
    //     times = 0;
    // }

    // 原始实现
    // for (int i = 0; i < sizeof(g_param_table) / sizeof(MotorParamItem); i++)
    // {
    //     MotorParamItem *item = &g_param_table[i];
    //     sendMotorCommand(can_id, item->can_cmd, {}, PRIORITY_NORMAL);
    // }
}

MotorParamItem *MotorParser::getItemByCanCmd(uint16_t can_cmd)
{
    for (int i = 0; i < sizeof(g_param_table) / sizeof(MotorParamItem); i++)
    {
        MotorParamItem *item = &g_param_table[i];
        if (item->can_cmd == can_cmd)
        {
            return item;
        }
    }
    return nullptr;
}

void MotorParser::receiveLoop()
{
    if (socket_fd < 0)
    {
        AINFO << "=======MotorParser::socket_fd=======" << socket_fd;
        // return {};
    }
    struct can_frame frame;
    while (running_)
    {
        ssize_t nbytes = 0;
        // if (ioctl(socket_fd, FIONREAD, &nbytes) == 0) {
        //     if (nbytes == 0) {
        //         printf("缓冲区为空\n");
        //         return {};
        //     } else {
        //         printf("缓冲区有 %d 字节数据待处理\n", nbytes);
        //     }
        // }

        // AINFO << "before read";
        nbytes = read(socket_fd, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            AINFO << "=======MotorParser::nbytes=======" << nbytes;
            continue;
        }

        std::ostringstream oss;
        oss << "receive data=" << frame.can_id;
        for (int i = 0; i < frame.can_dlc; i++)
        {
            oss << " " << std::hex << static_cast<int>(frame.data[i]);
        }
        AINFO << oss.str();

        // 喂狗
        if (frame.data[0] == 0x0A) {
            motor_heartbeat_[frame.can_id]->feed();
        }

        std::vector<uint8_t> response = std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
        MotorParamItem *item = getItemByCanCmd(frame.data[0]);
        if (item == nullptr)
        {
            AERROR << frame.can_id << "不存在！";
        }
        MotorData *motor_data = &motor_data_[frame.can_id];
        switch (item->type)
        {
        case PARAM_TYPE_INT32:
        {
            int32_t data = parseInt32(response);
            memcpy(PARAM_PTR(motor_data, item), &data, sizeof(int32_t));
            break;
        }
        case PARAM_TYPE_DOUBLE:
        {
            double data = static_cast<double>(parseInt32(response));
            memcpy(PARAM_PTR(motor_data, item), &data, sizeof(double));
            break;
        }
        case PARAM_TYPE_THETA:
        {
            double data = (parseInt32(response) / 100.0 / gearRatio) * 360.0;
            memcpy(PARAM_PTR(motor_data, item), &data, sizeof(double));
            break;
        }
        case PARAM_TYPE_CODERTHETA:
        {
            double data = (parseInt32(response) / 65536.0 / gearRatio) * 360.0;
            memcpy(PARAM_PTR(motor_data, item), &data, sizeof(double));
            break;
        }
        case PARAM_TYPE_STRING:
        {
            int32_t mode = parseInt32(response);
            switch (mode)
            {
            case -1:
                motor_data->mode = "Fault mode";
            case 0:
                motor_data->mode = "Stop mode";
            case 1:
                motor_data->mode = "Current mode";
            case 2:
                motor_data->mode = "Speed mode";
            case 3:
                motor_data->mode = "Position mode";
            default:
                motor_data->mode = "Unknown mode";
            }
            break;
        }
        case PARAM_TYPE_ERROR:
        {
            int32_t status = parseInt32(response);
            MotorErrorStatus err;
            err.bit0_softwareError = status & (1 << 0);
            err.bit1_overVoltage = status & (1 << 1);
            err.bit2_underVoltage = status & (1 << 2);
            err.bit4_startupError = status & (1 << 4);
            err.bit5_speedFeedbackError = status & (1 << 5);
            err.bit6_overCurrent = status & (1 << 6);
            err.bit7_softwareErrorOther = status & (1 << 7);
            err.bit16_encoderCommError = status & (1 << 16);
            err.bit17_motorOverTemp = status & (1 << 17);
            err.bit18_boardOverTemp = status & (1 << 18);
            err.bit19_driverChipError = status & (1 << 19);
            memcpy(PARAM_PTR(motor_data, item), &err, sizeof(MotorErrorStatus));
            break;
        }
        default:
            break;
        }
    };
}

void MotorParser::stopReceiveThread()
{
    running_ = false;
    if (read_thread_.joinable())
    {
        read_thread_.join();
    }
}

// 电机指令处理线程
void MotorParser::commandProcessingThread()
{
    while (running_)
    {
        MotorCommand command;
        bool has_command = false;

        {
            std::unique_lock<std::mutex> lock(queue_mtx_);
            cmd_cv_.wait(lock, [this]
                         { return !high_priority_queue_.empty() ||
                                  !normal_priority_queue_.empty() ||
                                  !running_; });

            if (!running_)
                break;

            // 优先处理高优先级指令
            if (!high_priority_queue_.empty())
            {
                command = high_priority_queue_.top();
                high_priority_queue_.pop();
                has_command = true;
            }
            else if (!normal_priority_queue_.empty())
            {
                command = normal_priority_queue_.front();
                normal_priority_queue_.pop();
                has_command = true;
            }
        }

        if (has_command)
        {
            // 获取开始时间点
            // auto start = std::chrono::steady_clock::now();
            executeCommand(command);
            // // 获取结束时间点
            // auto end = std::chrono::steady_clock::now();
            // // 计算时间间隔（以毫秒为单位）
            // auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // 输出时间间隔
            // AINFO << "发送命令耗时: " << duration_ms.count() << " 毫秒";
            // AINFO << "发送命令: " << command.can_id << " -------" << command.cmd;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

void MotorParser::executeCommand(const MotorCommand &command)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (socket_fd < 0)
    {
        AERROR << "Socket not initialized";
        return;
    }

    struct can_frame frame;
    frame.can_id = command.can_id;

    size_t total_len = std::min(command.data.size() + 1, static_cast<size_t>(8));
    frame.can_dlc = static_cast<__u8>(total_len);

    frame.data[0] = command.cmd;

    for (size_t i = 0; i < total_len - 1; ++i)
    {
        frame.data[i + 1] = command.data[i];
    }

    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame))
    {
        AERROR << "Failed to send command: " << static_cast<int>(command.cmd);
    }
}

void MotorParser::sendMotorCommand(int can_id, uint8_t cmd, const std::vector<uint8_t> &data, CommandPriority priority)
{
    MotorCommand command;
    command.can_id = can_id;
    command.cmd = cmd;
    command.data = data;
    command.priority = priority;

    std::lock_guard<std::mutex> lock(queue_mtx_);
    // 检查队列长度限制
    if (priority == PRIORITY_HIGH)
    {
        if (high_priority_queue_.size() >= MAX_HIGH_PRIORITY_QUEUE_SIZE)
        {
            AERROR << "High priority queue full! Discarding command: "
                   << static_cast<int>(cmd) << " for motor " << can_id;
            return; // 队列满时丢弃新指令
        }
        high_priority_queue_.push(command);
    }
    else
    {
        if (normal_priority_queue_.size() >= MAX_NORMAL_PRIORITY_QUEUE_SIZE)
        {
            AERROR << "Normal priority queue full! Discarding command: "
                   << static_cast<int>(cmd) << " for motor " << can_id;
            return; // 队列满时丢弃新指令
        }
        normal_priority_queue_.push(command);
    }
    cmd_cv_.notify_one(); // 唤醒指令处理线程
}

void MotorParser::heartbeatTimeout(int can_id) {
    MotorData* motor_data = &motor_data_[can_id];
    motor_data->disconnect = true;
}

void MotorParser::heartbeatRecover(int can_id) {
    MotorData* motor_data = &motor_data_[can_id];
    motor_data->disconnect = false;
}