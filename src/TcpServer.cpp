#include "TcpServer.h"
#include <iostream>
#include <cstring>
#include <log.h>
#include <set>
#include <cerrno>

uint64_t htonll(uint64_t value) {
    if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) {
        return (static_cast<uint64_t>(htonl(value & 0xFFFFFFFFULL)) << 32) |
               htonl(value >> 32);
    } else {
        return value;
    }
}

uint64_t ntohll(uint64_t value) {
    if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) {
        return (static_cast<uint64_t>(ntohl(value & 0xFFFFFFFFULL)) << 32) |
               ntohl(value >> 32);
    } else {
        return value;
    }
}

// float: host -> network
uint32_t htonf(float value) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");

    uint32_t temp;
    memcpy(&temp, &value, sizeof(temp));
    return htonl(temp);
}

// float: network -> host
float ntohf(uint32_t net_value) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");

    uint32_t host_value = ntohl(net_value);
    float result;
    memcpy(&result, &host_value, sizeof(result));
    return result;
}

TcpServer::TcpServer(EventBus& bus) : bus_(bus) {
    // 订阅控制命令事件
    bus_.subscribe<Server_Info>("to_tcp",
        [this](const Server_Info& info){ returnTcpData(info); });
}

TcpServer::~TcpServer() {
    stop();
}

bool TcpServer::start(int port) {
    // 1. 创建 socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        AERROR << "create socket failed";
        return false;
    }

    // 2. 绑定地址和端口
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // 0.0.0.0
    server_addr.sin_port = htons(port);

    if (bind(server_fd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        AERROR << "bind failed";
        close(server_fd);
        return false;
    }

    // 3. 开始监听
    if (listen(server_fd, 5) < 0) {
        AERROR << "listen failed";
        close(server_fd);
        return false;
    }

    AINFO << "TCP server listening on port 8080...";

    running_ = true;
    serverThread_ = std::thread([this]{ serverLoop(); });
    serverThread_.join();
    return true;
}

void TcpServer::stop() {
    running_ = false;
    if (serverThread_.joinable()) serverThread_.join();
    close(server_fd);
}

void TcpServer::serverLoop() {
    while (running_) {
        AINFO << "Waiting for new client..." << server_fd <<std::endl;
        // 4. 接收客户端连接
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            AERROR << "accept failed" << strerror(errno);
            continue;
        }
        AINFO << "Client connected.";
        connections.insert(client_fd);
        std::thread(&TcpServer::handleClient, this, client_fd).detach();
    }
}

void TcpServer::handleClient(int client_fd) {
    uint8_t receive_buffer[TCP_MAX_BUFFER_LENGTH];
    while (running_) {
        memset(receive_buffer, 0, sizeof(receive_buffer));
        ssize_t n = recv(client_fd, receive_buffer, sizeof(receive_buffer) - 1, 0);
        if (n <= 0) {
            AINFO << "Client disconnected.";
            break; // 跳出内层循环，重新accept新客户端
        }

        std::cout << "Received: " << receive_buffer << std::endl;
        Server_Ctrl ctl;
        memset(&ctl, 0, sizeof(Server_Ctrl));
        parseServerCtrl(receive_buffer, n, ctl);

        bus_.publish("from_tcp", ctl);
    }
    close(client_fd);
    connections.erase(client_fd);
}

void TcpServer::returnTcpData(const Server_Info &info) {
    AINFO << "return bus";
    // 回发数据
    size_t len = packServerInfo(info, send_buffer);
    for (auto& fd : connections) {
        send(fd, send_buffer, len, 0);
    }
}

// 解包
bool TcpServer::parseServerCtrl(const uint8_t* buf, size_t len, Server_Ctrl& out) {
    // 1. 长度校验
    if (len < 24) {
        return false;
    }

    size_t offset = 0;

    // 2. 消息类型
    uint16_t net_msg_type;
    memcpy(&net_msg_type, buf + offset, 2);
    uint16_t msg_type = ntohs(net_msg_type);
    offset += 2;

    if (msg_type != 1) {
        return false;
    }

    // 3. 消息长度
    uint16_t net_msg_len;
    memcpy(&net_msg_len, buf + offset, 2);
    uint16_t msg_len = ntohs(net_msg_len);
    offset += 2;

    if (msg_len != 20) {
        return false;
    }

    // 4. 控制模式
    uint16_t net_ctrl_mode;
    memcpy(&net_ctrl_mode, buf + offset, 2);
    out.ctrl_mode = ntohs(net_ctrl_mode);
    offset += 2;

    // 5. 左侧伸出百分比 float
    uint32_t net_f;
    memcpy(&net_f, buf + offset, 4);
    net_f = ntohl(net_f);
    memcpy(&out.ext_left, &net_f, 4);
    offset += 4;

    // 6. 右侧伸出百分比 float
    memcpy(&net_f, buf + offset, 4);
    net_f = ntohl(net_f);
    memcpy(&out.ext_right, &net_f, 4);
    offset += 4;

    // 7. 下位机关机
    uint16_t net_shutdown;
    memcpy(&net_shutdown, buf + offset, 2);
    out.shutdown = ntohs(net_shutdown);
    offset += 2;

    // 8. 时间戳 uint64
    uint64_t net_ts;
    memcpy(&net_ts, buf + offset, 8);
    out.timestamp = ntohll(net_ts);
    offset += 8;

    return true;
}

// 封包
size_t TcpServer::packServerInfo(const Server_Info& server_info, uint8_t* buffer) {
    size_t offset = 0;

    // 消息类型 (uint16)
    uint16_t msg_type = 1;
    uint16_t msg_type_network = htons(msg_type);
    memcpy(buffer + offset, &msg_type_network, sizeof(msg_type_network));
    offset += 2;

    // 消息长度 (uint16)
    uint16_t msg_length = 58 + server_info.motor_num * 2; // 58 字节 + 每个电机状态 2 字节
    uint16_t msg_length_network = htons(msg_length);
    memcpy(buffer + offset, &msg_length_network, sizeof(msg_length_network));
    offset += 2;

    // 当前航速 (float)
    uint32_t speed_network = htonf(server_info.speed);
    memcpy(buffer + offset, &speed_network, sizeof(speed_network));
    offset += sizeof(speed_network);

    // 左侧截流板伸出阈值 (float)
    uint32_t ext_left_limit_network = htonf(server_info.ext_left_limit);
    memcpy(buffer + offset, &ext_left_limit_network, sizeof(ext_left_limit_network));
    offset += sizeof(ext_left_limit_network);

    // 右侧截流板伸出阈值 (float)
    uint32_t ext_right_limit_network = htonf(server_info.ext_right_limit);
    memcpy(buffer + offset, &ext_right_limit_network, sizeof(ext_right_limit_network));
    offset += sizeof(ext_right_limit_network);

    // 左侧截流板当前伸出百分比 (float)
    uint32_t ext_left_network = htonf(server_info.ext_left);
    memcpy(buffer + offset, &ext_left_network, sizeof(ext_left_network));
    offset += sizeof(ext_left_network);

    // 右侧截流板当前伸出百分比 (float)
    uint32_t ext_right_network = htonf(server_info.ext_right);
    memcpy(buffer + offset, &ext_right_network, sizeof(ext_right_network));
    offset += sizeof(ext_right_network);

    // 电机数量 (uint16)
    uint16_t motor_num_network = htons(server_info.motor_num);
    memcpy(buffer + offset, &motor_num_network, sizeof(motor_num_network));
    offset += sizeof(motor_num_network);

    // 电机状态 (uint16 数组)
    for (size_t i = 0; i < server_info.motor_num; ++i) {
        uint16_t motor_state_network = htons(server_info.motor_state[i]);
        memcpy(buffer + offset, &motor_state_network, sizeof(motor_state_network));
        offset += sizeof(motor_state_network);
    }

    // 下位机状态码 (uint16)
    uint16_t pc_state_network = htons(server_info.pc_state);
    memcpy(buffer + offset, &pc_state_network, sizeof(pc_state_network));
    offset += sizeof(pc_state_network);

    // 船舶当前艏向角 (float)
    uint32_t heading_network = htonf(server_info.heading);
    memcpy(buffer + offset, &heading_network, sizeof(heading_network));
    offset += sizeof(heading_network);

    // 船舶当前纵倾角 (float)
    uint32_t pitch_network = htonf(server_info.pitch);
    memcpy(buffer + offset, &pitch_network, sizeof(pitch_network));
    offset += sizeof(pitch_network);

    // 船舶当前横倾角 (float)
    uint32_t roll_network = htonf(server_info.roll);
    memcpy(buffer + offset, &roll_network, sizeof(roll_network));
    offset += sizeof(roll_network);

    // 当前控制模式 (uint16)
    uint16_t control_mode_network = htons(0);  // 假设控制模式为0
    memcpy(buffer + offset, &control_mode_network, sizeof(control_mode_network));
    offset += sizeof(control_mode_network);

    // 时间戳 (uint64)
    uint64_t timestamp_network = htonll(server_info.timestamp);
    memcpy(buffer + offset, &timestamp_network, sizeof(timestamp_network));
    offset += sizeof(timestamp_network);

    return offset;  // 返回实际填充的字节数
}

