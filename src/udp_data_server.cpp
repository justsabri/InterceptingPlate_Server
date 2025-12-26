#include "udp_data_server.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctime>
#include <cmath>
#include <sstream>
#include <errno.h>

UdpDataServer::UdpDataServer(int port_num) : port(25012), is_running(false), server_fd(-1)
{
    // 初始化当前数据
    memset(&current_data, 0, sizeof(current_data));
    memset(&server_addr, 0, sizeof(server_addr));

    AINFO << "UDP数据服务器创建，将监听端口: " << port;
}

UdpDataServer::~UdpDataServer()
{
    stop();
}

bool UdpDataServer::start()
{
    if (is_running)
    {
        AINFO << "服务器已经在运行中";
        return true;
    }

    AINFO << "正在启动UDP数据服务器...";

    // 1. 创建UDP套接字
    server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_fd < 0)
    {
        AERROR << "创建套接字失败: " << strerror(errno);
        return false;
    }

    // 2. 设置套接字选项
    int optval = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0)
    {
        AERROR << "设置地址重用选项失败: " << strerror(errno);
        // 继续执行，这不是致命错误
    }

    // 3. 允许接收广播
    optval = 1; // 允许接收广播
    if (setsockopt(server_fd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval)) < 0)
    {
        AERROR << "设置广播接收选项失败: " << strerror(errno);
        // 继续执行，这不是致命错误
    }

    // 4. 设置接收超时（用于优雅退出）
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        AERROR << "设置接收超时失败: " << strerror(errno);
    }

    // 5. 配置服务器地址
    memset(&server_addr, 0, sizeof(server_addr)); // 清空结构体
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网络接口
    server_addr.sin_port = htons(port); // 设置端口为25012

    // 6. 绑定端口
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        AERROR << "绑定端口 25012 失败: " << strerror(errno);
        close(server_fd);
        server_fd = -1;
        return false;
    }

    // 6. 启动接收线程
    is_running = true;
    receiver_thread = std::thread(&UdpDataServer::receiverLoop, this);

    AINFO << "UDP数据服务器启动成功！正在监听端口: " << port;

    return true;
}

void UdpDataServer::stop()
{
    if (!is_running)
    {
        return;
    }

    AINFO << "正在停止UDP数据服务器...";
    is_running = false;

    // 关闭套接字以唤醒阻塞的recvfrom
    if (server_fd >= 0)
    {
        shutdown(server_fd, SHUT_RDWR);
        close(server_fd);
        server_fd = -1;
    }

    // 等待接收线程结束
    if (receiver_thread.joinable())
    {
        receiver_thread.join();
    }

    AINFO << "UDP数据服务器已停止";
}

void UdpDataServer::receiverLoop()
{
    uint8_t buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    AINFO << "接收线程启动，等待数据...";

    while (is_running)
    {
        // 接收UDP数据报
        ssize_t bytes_received = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &client_addr_len);

        AINFO << "接收数据..." << bytes_received << " 字节";
        if (bytes_received < 0)
        {
            // 检查是否超时（这是正常的，用于检查is_running状态）
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            // 其他错误且服务器仍在运行，则报告错误
            if (is_running)
            {
                AERROR << "接收数据错误: " << strerror(errno);
            }
            continue;
        }

        // 获取客户端信息
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
        uint16_t client_port = ntohs(client_addr.sin_port);

        // 打印客户端连接信息和原始报文
        buffer[bytes_received] = '\0'; // 确保以字符串形式输出
        std::stringstream hex_stream;
        hex_stream << std::hex << std::uppercase << std::setfill('0');
        for (int i = 0; i < bytes_received; ++i)
        {
            hex_stream << std::setw(2) << static_cast<int>(buffer[i]) << " ";
        }

        AINFO << "接收到来自客户端 " << client_ip << ":" << client_port << " 的数据，长度: " << bytes_received << " 字节";
        AINFO << "原始报文(十六进制): " << hex_stream.str();
        AINFO << "原始报文: " << buffer;
        //-------------------------------------------------------------------
        // 1-下面将报文进行解析:

        //-------------------------------------------------------------------
        
        // 解析数据
        ImuData new_data = parseData(buffer, bytes_received);
        // 只有当解析成功时才更新数据（检查gps_time是否不为空）
        if (!new_data.gps_time.empty()) {
            // 线程安全地更新当前数据
            std::lock_guard<std::mutex> lock(data_mutex);
            current_data = new_data;
            AINFO << "更新IMU数据: " << current_data.gps_time;
        } else {
            AINFO << "解析失败，跳过此数据包";
        }

    }
}

float UdpDataServer::bigEndianToFloat(const uint8_t *data)
{
    union
    {
        uint32_t i;
        float f;
    } converter;

    // 将大端序的4字节转换为32位整数
    converter.i = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) | (static_cast<uint32_t>(data[2]) << 8) |
                  static_cast<uint32_t>(data[3]);
    return converter.f;
}

uint32_t UdpDataServer::bigEndianToUint32(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) | (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

ImuData UdpDataServer::parseData(uint8_t *buffer, size_t length)
{
    ImuData data = {0};
    
    // 检查最小长度
    if (length < 20) {
        AINFO << "数据包长度太短: " << length << " 字节";
        return data;
    }
    
    // 1. 将二进制数据转换为字符串以便处理
    std::string received_data(reinterpret_cast<char*>(buffer), length);
    AINFO << "原始数据字符串: " << received_data;
    
    // 2. 查找GPFPD协议头
    size_t gpfpd_pos = received_data.find("$GPFPD");
    if (gpfpd_pos == std::string::npos) {
        AINFO << "未找到协议头 $GPFPD";
        return data;
    }
    
    // 3. 查找校验和标记 "*"
    size_t asterisk_pos = received_data.find("*", gpfpd_pos);
    if (asterisk_pos == std::string::npos) {
        AINFO << "未找到校验和标记 *";
        return data;
    }
    
    // 4. 检查是否有足够的字节用于校验和和结束符
    if (length < asterisk_pos + 3) {
        AINFO << "数据不完整，缺少校验和或结束符";
        return data;
    }
    
    // 5. 计算校验和（从'$'后到'*'前的所有字符）
    uint8_t calculated_checksum = 0;
    for (size_t i = gpfpd_pos + 1; i < asterisk_pos; i++) {
        calculated_checksum ^= static_cast<uint8_t>(received_data[i]);
    }
    
    // 6. 提取接收到的校验和（两位十六进制）
    std::string checksum_str = received_data.substr(asterisk_pos + 1, 2);
    uint8_t received_checksum = static_cast<uint8_t>(
        std::stoi(checksum_str, nullptr, 16));
    
    if (calculated_checksum != received_checksum) {
        AINFO << "校验和验证失败: 计算=" << std::hex << static_cast<int>(calculated_checksum)
              << ", 接收=" << static_cast<int>(received_checksum);
        return data;
    }
    
    AINFO << "校验和验证通过: 0x" << std::hex << static_cast<int>(calculated_checksum);
    
    // 7. 检查结束符CRLF
    if (asterisk_pos + 3 >= length || 
        received_data[asterisk_pos + 3] != '\r' || 
        (asterisk_pos + 4 < length && received_data[asterisk_pos + 4] != '\n')) {
        AINFO << "结束符不完整或格式错误";
        // 这里不直接返回，因为可能有些实现只发送CR
    }
    
    // 8. 提取NMEA数据部分（从"$GPFPD"到"*HH"之前）
    std::string nmea_data = received_data.substr(gpfpd_pos, asterisk_pos - gpfpd_pos);
    AINFO << "NMEA数据: " << nmea_data;
    
    // 9. 解析数据字段
    // 去掉"$GPFPD,"
    std::string fields_str = nmea_data.substr(7);
    
    std::stringstream ss(fields_str);
    std::string item;
    std::vector<std::string> fields;
    
    while (std::getline(ss, item, ',')) {
        fields.push_back(item);
    }
    
    // 根据图片1，GPFPD消息应该有15个字段（不包括协议头和校验和）
    if (fields.size() < 15) {
        AINFO << "字段数量不足: " << fields.size() << "，应有15个字段";
        AINFO << "字段列表: ";
        for (size_t i = 0; i < fields.size(); i++) {
            AINFO << "  [" << i << "] " << fields[i];
        }
        return data;
    }
    
    try {
    int gps_week = std::stoi(fields[0]);       // GPS周数
    double gps_seconds = std::stod(fields[1]); // GPS秒数（可能有小数部分）
    
    // GPS时间转换：从GPS时间转换为UTC时间
    // GPS时间起始于1980年1月6日 00:00:00 UTC
    // 1. 计算从GPS起始时间到当前的总秒数
    double total_gps_seconds = gps_week * 7.0 * 24.0 * 3600.0 + gps_seconds + 8 * 3600;
    
    // 2. GPS时间与UTC时间之间的闰秒差（随时间变化，这里使用2025年的差值）
    // 注意：闰秒会变化，这里使用2025年的闰秒差为-18秒
    int leap_seconds = -18; // 2025年GPS-UTC的闰秒差
    
    // 3. 计算从Unix时间戳起始(1970-01-01)到GPS时间起始(1980-01-06)的秒数
    // 1980-01-06 00:00:00 UTC 到 1970-01-01 00:00:00 UTC 的秒数
    long gps_epoch_to_unix = 315964800L; // 这个值是固定的
    
    // 4. 计算Unix时间戳（带小数）
    double unix_timestamp = gps_epoch_to_unix + total_gps_seconds + leap_seconds;
    
    // 5. 将Unix时间戳转换为time_t（整数秒）和毫秒部分
    time_t time_seconds = static_cast<time_t>(unix_timestamp);
    double fractional_seconds = unix_timestamp - time_seconds;
    
    // 6. 转换为UTC时间结构
    struct tm* utc_time = gmtime(&time_seconds);
    if (!utc_time) {
        AERROR << "时间转换失败";
        return data;
    }
    
    // 7. 格式化为yyyymmddhhmmss.ss字符串
    char time_str[30]; // 足够存储yyyymmddhhmmss.ss
    int year = utc_time->tm_year + 1900;  // tm_year是从1900年开始的
    int month = utc_time->tm_mon + 1;     // tm_mon是0-11
    int day = utc_time->tm_mday;
    int hour = utc_time->tm_hour;
    int minute = utc_time->tm_min;
    int second = utc_time->tm_sec;
    
    // 格式化整数部分
    snprintf(time_str, sizeof(time_str), "%04d%02d%02d%02d%02d%02d", 
             year, month, day, hour, minute, second);
    
    // 添加小数秒部分（两位小数）
    std::stringstream time_ss;
    time_ss << time_str << "." 
            << std::setw(2) << std::setfill('0') 
            << static_cast<int>(fractional_seconds * 100);
    
        data.gps_time = time_ss.str();
        data.heading = std::stod(fields[2]);        // 航向角
        data.pitch = std::stod(fields[3]);         // 俯仰角
        data.roll = std::stod(fields[4]);          // 横滚角
        data.latitude = std::stod(fields[5]);      // 纬度
        data.longitude = std::stod(fields[6]);     // 经度
        double ve = std::stod(fields[8]);           // 东向速度
        double vn = std::stod(fields[9]);           // 北向速度
        
        
        // 计算水平速度（节），1米/秒 = 1.943844节
        data.speed = std::sqrt(ve * ve + vn * vn) * 1.943844;
        
        AINFO << "解析成功:";
        AINFO << "  时间字符串: " << data.gps_time;
        AINFO << "  纬度: " << data.latitude;
        AINFO << "  经度: " << data.longitude;
        AINFO << "  航向角: " << data.heading;
        AINFO << "  俯仰角: " << data.pitch;
        AINFO << "  横滚角: " << data.roll;
        AINFO << "  速度 (kn): " << data.speed;
        
    } catch (const std::exception& e) {
        AERROR << "解析数据字段时出错: " << e.what();
        AINFO << "错误字段详情: ";
        for (size_t i = 0; i < fields.size(); i++) {
            AINFO << "  Field[" << i << "]: " << fields[i];
        }
        return data;
    }
    AINFO << "数据解析完成";
    return data;
}

ImuData UdpDataServer::getData()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    AINFO << "获取当前数据:";
    AINFO << "  时间字符串: " << current_data.gps_time;
    return current_data;
}
