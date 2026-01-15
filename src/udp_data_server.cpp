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
#include <iomanip>
#include <errno.h>

UdpDataServer::UdpDataServer(int port_num)
    : port(9090),
      is_running(false),
      server_fd(-1),
      current_data() {

    // 初始化服务器地址
    memset(&server_addr, 0, sizeof(server_addr));

    AINFO << "UDP数据服务器创建，将监听端口: " << port ;
}

UdpDataServer::~UdpDataServer() {
    stop();
}

bool UdpDataServer::start() {
    if (is_running) {
        AINFO << "服务器已经在运行中" ;
        return true;
    }

    AINFO << "正在启动UDP数据服务器..." ;

    // 1. 创建UDP套接字
    server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_fd < 0) {
        AERROR << "创建套接字失败: " << strerror(errno) ;
        return false;
    }

    // 2. 设置套接字选项
    int optval = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        AERROR << "设置地址重用选项失败: " << strerror(errno);
        // 继续执行，这不是致命错误
    }

    // 3. 设置接收超时（用于优雅退出）
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        AERROR << "设置接收超时失败: " << strerror(errno) ;
    }

    // 4. 配置服务器地址
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.201");  // 监听所有网络接口
    server_addr.sin_port = htons(port);

    // 5. 绑定端口
    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        AERROR << "绑定端口 " << port << " 失败: " << strerror(errno) ;
        close(server_fd);
        server_fd = -1;
        return false;
    }

    // 6. 启动接收线程
    is_running = true;
    receiver_thread = std::thread(&UdpDataServer::receiverLoop, this);

    AINFO << "UDP数据服务器启动成功！正在监听端口: " << port ;
    AINFO << "数据格式: 12个字段，总共56字节" ;
    AINFO << "字段顺序: 横倾角→纵倾角→舵角→航速→主机转速→时间戳→经度→纬度→左机转速→右机转速→左机档位→右机档位" ;
    AINFO << "字段长度: 前10个字段4字节/个，经度纬度8字节/个" ;

    return true;
}

void UdpDataServer::stop() {
    if (!is_running) {
        return;
    }

    AINFO << "正在停止UDP数据服务器...";
    is_running = false;

    // 关闭套接字以唤醒阻塞的recvfrom
    if (server_fd >= 0) {
        shutdown(server_fd, SHUT_RDWR);
        close(server_fd);
        server_fd = -1;
    }

    // 等待接收线程结束
    if (receiver_thread.joinable()) {
        receiver_thread.join();
    }

    AINFO << "UDP数据服务器已停止" ;
}

void UdpDataServer::receiverLoop() {
    uint8_t buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    AINFO << "接收线程启动，等待数据..." ;

    while (is_running) {
        // 接收UDP数据报
        ssize_t bytes_received = recvfrom(server_fd, buffer, sizeof(buffer), 0,
                                        (struct sockaddr*)&client_addr, &client_addr_len);

        AINFO << "接收数据..." << bytes_received << " 字节";
        if (bytes_received < 0) {
            // 检查是否超时（这是正常的，用于检查is_running状态）
            if (errno == EAGAIN || errno == EWOULDBLOCK) {

                continue;
            }
            // 其他错误且服务器仍在运行，则报告错误
            if (is_running) {
                AERROR << "接收数据错误: " << strerror(errno) ;
            }
            continue;
        }

        // 获取客户端信息
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
        uint16_t client_port = ntohs(client_addr.sin_port);

        // 打印客户端连接信息和原始报文
        std::stringstream hex_stream;
        hex_stream << std::hex << std::uppercase << std::setfill('0');
        for (int i = 0; i < bytes_received; ++i) {
            hex_stream << std::setw(2) << static_cast<int>(buffer[i]) << " ";
        }

        AINFO << "接收到来自客户端 " << client_ip << ":" << client_port
              << " 的数据，长度: " << bytes_received << " 字节";
        AINFO << "原始报文(十六进制): " << hex_stream.str();


        // 检查数据长度（根据图片应为24字节）
        // if (bytes_received != sizeof(ImuData)) {
        //     AERROR << "数据长度不匹配！期望: " << sizeof(ImuData)
        //               << " 字节，实际收到: " << bytes_received << " 字节";
        //     continue;
        // }
        
        // 解析数据
        ImuData new_data = parseData(buffer, bytes_received);

        // 验证数据范围（根据图片中的取值范围）
        // if (!validateData(new_data)) {
        //     AERROR << "数据验证失败，数据可能无效" ;
        //     continue;
        // }

        // 线程安全地更新当前数据
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_data = new_data;
        }

        AINFO << "接收到数据"<< " - 横倾角:" << new_data.roll << "° 纵倾角:" << new_data.pitch << "°"
          << " 航速:" << new_data.speed << "kn 转速:" << new_data.rpm << "rpm 时间戳:"<< new_data.timestamp
          << " 舵角:" << new_data.rudder << "° 经度:" << std::fixed << std::setprecision(8) << new_data.longitude << "° 纬度:" << std::fixed << std::setprecision(8) << new_data.latitude << "°"
          << " 左机转速:" << new_data.left_rpm << "rpm 右机转速:" << new_data.right_rpm << "rpm"
          << " 左机档位:" << new_data.left_gear << " 右机档位:" << new_data.right_gear;
        std::cout << "经度:" << std::fixed << std::setprecision(8) << new_data.longitude << "纬度:" << std::fixed << std::setprecision(8) << new_data.latitude;

    }
}

float UdpDataServer::bigEndianToFloat(const uint8_t* data) {
    union {
        uint32_t i;
        float f;
    } converter;

    // 将大端序的4字节转换为32位整数
    converter.i = (static_cast<uint32_t>(data[0]) << 24) |
                  (static_cast<uint32_t>(data[1]) << 16) |
                  (static_cast<uint32_t>(data[2]) << 8) |
                  static_cast<uint32_t>(data[3]);
    return converter.f;
}

uint32_t UdpDataServer::bigEndianToUint32(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

double UdpDataServer::bigEndianToDouble(const uint8_t* data) {
    union {
        uint64_t i;
        double d;
    } converter;

    // 将大端序的8字节转换为64位整数
    converter.i = (static_cast<uint64_t>(data[0]) << 56) |
                  (static_cast<uint64_t>(data[1]) << 48) |
                  (static_cast<uint64_t>(data[2]) << 40) |
                  (static_cast<uint64_t>(data[3]) << 32) |
                  (static_cast<uint64_t>(data[4]) << 24) |
                  (static_cast<uint64_t>(data[5]) << 16) |
                  (static_cast<uint64_t>(data[6]) << 8) |
                  static_cast<uint64_t>(data[7]);

    return converter.d;
}



ImuData UdpDataServer::parseData(const uint8_t* buffer, size_t length) {
    ImuData data = {0};
    
    // 检查是否有足够的数据（56字节，对应12个字段）
    if (length < 56) {
        AERROR << "数据长度不足，期望: 56 字节，实际收到: " << length << " 字节";
        return data;
    }
    
    // 按照表格顺序解析字段（全部为大端字节序）
    int offset = 0;
    data.roll = bigEndianToFloat(buffer + offset);       // 0-3字节: 横倾角
    offset += 4;
    data.pitch = bigEndianToFloat(buffer + offset);      // 4-7字节: 纵倾角
    offset += 4;
    data.rudder = bigEndianToFloat(buffer + offset);     // 8-11字节: 舵角
    offset += 4;
    data.speed = bigEndianToFloat(buffer + offset);      // 12-15字节: 航速
    offset += 4;
    data.rpm = bigEndianToFloat(buffer + offset);        // 16-19字节: 主机转速
    offset += 4;
    data.timestamp = bigEndianToUint32(buffer + offset); // 20-23字节: UNIX时间戳
    offset += 4;
    data.longitude = bigEndianToDouble(buffer + offset); // 24-31字节: 经度
    offset += 8;
    data.latitude = bigEndianToDouble(buffer + offset);  // 32-39字节: 纬度
    offset += 8;
    data.left_rpm = bigEndianToFloat(buffer + offset);   // 40-43字节: 左机转速
    offset += 4;
    data.right_rpm = bigEndianToFloat(buffer + offset);  // 44-47字节: 右机转速
    offset += 4;
    data.left_gear = bigEndianToUint32(buffer + offset);  // 48-51字节: 左机档位
    offset += 4;
    data.right_gear = bigEndianToUint32(buffer + offset); // 52-55字节: 右机档位
    
    return data;
}

bool UdpDataServer::validateData(const ImuData& data) {
    // 根据图片表格中的取值范围进行验证
    if (data.roll < -40.0f || data.roll > 40.0f) {
        AERROR << "横倾角超出范围: " << data.roll << "° (范围: -40°~40°)"  ;
        return false;
    }
    
    if (data.pitch < -30.0f || data.pitch > 30.0f) {
        AERROR << "纵倾角超出范围: " << data.pitch << "° (范围: -30°~30°)" ;
        return false;
    }
    
    if (data.rudder < -30.0f || data.rudder > 30.0f) {
        AERROR << "舵角超出范围: " << data.rudder << "° (范围: -30°~30°)" ;
        return false;
    }
    
    if (data.speed < 0.0f || data.speed > 80.0f) {
        AERROR << "航速超出范围: " << data.speed << "kn (范围: 0~80kn)" ;
        return false;
    }
    
    if (data.rpm < 0.0f || data.rpm > 5000.0f) {
        AERROR << "主机转速超出范围: " << data.rpm << "rpm (范围: 0~5000rpm)" ;
        return false;
    }
    
    // 时间戳验证（通常不早于2020年，不晚于未来1小时）
    std::time_t current_time = std::time(nullptr);
    if (data.timestamp < 1577836800) { // 2020-01-01 00:00:00
        AERROR << "时间戳异常，可能无效: " << data.timestamp ;
        return false;
    }
    
    if (data.timestamp > current_time + 3600) { // 不超过未来1小时
        AERROR << "时间戳在未来，可能无效: " << data.timestamp ;
        return false;
    }
    
    // 新增字段验证
    if (data.longitude < -180.0 || data.longitude > 180.0) {
        AERROR << "经度超出范围: " << std::fixed << std::setprecision(8) << data.longitude << "° (范围: -180°~180°)" ;
        return false;
    }
    
    if (data.latitude < -90.0 || data.latitude > 90.0) {
        AERROR << "纬度超出范围: " << std::fixed << std::setprecision(8) << data.latitude << "° (范围: -90°~90°)" ;
        return false;
    }
    
    if (data.left_rpm < 0.0f || data.left_rpm > 5000.0f) {
        AERROR << "左机转速超出范围: " << data.left_rpm << "rpm (范围: 0~5000rpm)" ;
        return false;
    }
    
    if (data.right_rpm < 0.0f || data.right_rpm > 5000.0f) {
        AERROR << "右机转速超出范围: " << data.right_rpm << "rpm (范围: 0~5000rpm)" ;
        return false;
    }
    
    if (data.left_gear < 0 || data.left_gear > 3) {
        AERROR << "左机档位超出范围: " << data.left_gear << " (范围: 0~3)" ;
        return false;
    }
    
    if (data.right_gear < 0 || data.right_gear > 3) {
        AERROR << "右机档位超出范围: " << data.right_gear << " (范围: 0~3)" ;
        return false;
    }

    return true;
}

ImuData UdpDataServer::getData() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_data;
}

std::string UdpDataServer::getStatus() const {
    std::stringstream ss;

    if (is_running) {
        ss << "运行中 (端口: " << port << ")";

        // 获取当前数据的时间信息
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(data_mutex));
        if (current_data.timestamp > 0) {
            std::time_t time_val = current_data.timestamp;
            char time_buf[64];
            std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", std::localtime(&time_val));
            ss << " - 最新数据时间: " << time_buf;
        }
    } else {
        ss << "已停止";
    }

    return ss.str();
}