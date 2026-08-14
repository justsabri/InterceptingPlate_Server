#include "TcpServer.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <log.h>
#include <set>

namespace {

Server_Ctrl toServerCtrl(const protocol_15m::ControlCommand& command) {
    Server_Ctrl ctrl{};
    ctrl.ctrl_mode = command.control_mode;
    ctrl.auto_mode_param = command.auto_mode_param;
    ctrl.ext_left = command.manual_left_extend;
    ctrl.ext_right = command.manual_right_extend;
    ctrl.timestamp = command.timestamp_ms;
    return ctrl;
}

protocol_15m::InterceptorStatus toInterceptorStatus(const Server_Info& info) {
    protocol_15m::InterceptorStatus status{};
    status.current_speed = info.speed;
    status.left_extend_threshold = info.ext_left_limit;
    status.right_extend_threshold = info.ext_right_limit;
    status.left_current_extend = info.ext_left;
    status.right_current_extend = info.ext_right;
    status.motor_count = info.motor_num;
    status.motor1_status = info.motor_state.size() > 0 ? info.motor_state[0] : 101;
    status.motor2_status = info.motor_state.size() > 1 ? info.motor_state[1] : 101;
    status.motor3_status = info.motor_state.size() > 2 ? info.motor_state[2] : 101;
    status.motor4_status = info.motor_state.size() > 3 ? info.motor_state[3] : 101;
    status.imu_status = info.imu_state;
    status.slave_status = info.pc_state;
    status.current_pitch = info.pitch;
    status.current_roll = info.roll;
    return status;
}

}  // namespace

TcpServer::TcpServer(EventBus& bus) : bus_(bus) {
    bus_.subscribe<Server_Info>("to_tcp",
        [this](const Server_Info& info){ returnTcpData(info); });
}

TcpServer::~TcpServer() {
    stop();
}

bool TcpServer::start(int port) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        AERROR << "create socket failed";
        return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(server_fd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        AERROR << "bind failed";
        close(server_fd);
        return false;
    }

    if (listen(server_fd, 5) < 0) {
        AERROR << "listen failed";
        close(server_fd);
        return false;
    }

    AINFO << "TCP server listening on port " << port;

    running_ = true;
    serverThread_ = std::thread([this]{ serverLoop(); });
    serverThread_.join();
    return true;
}

void TcpServer::stop() {
    running_ = false;
    if (serverThread_.joinable()) serverThread_.join();
    if (server_fd >= 0) close(server_fd);
}

void TcpServer::serverLoop() {
    while (running_) {
        AINFO << "Waiting for new client..." << server_fd << std::endl;
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
        ssize_t n = recv(client_fd, receive_buffer, sizeof(receive_buffer), 0);
        if (n <= 0) {
            AINFO << "Client disconnected.";
            break;
        }

        protocol_15m::FrameHeader header;
        protocol_15m::DecodeResult header_result =
            protocol_15m::decodeHeader(receive_buffer, static_cast<size_t>(n), header);
        if (!header_result.ok) {
            AERROR << "15m TCP frame header invalid: " << header_result.error;
            continue;
        }

        if (header.msg_type == protocol_15m::kMsgTypeControlCommand) {
            Server_Ctrl ctl{};
            protocol_15m::DecodeResult result =
                parseServerCtrl(receive_buffer, static_cast<size_t>(n), ctl);
            if (!result.ok) {
                AERROR << "15m control command invalid: " << result.error;
                continue;
            }
            bus_.publish("from_tcp", ctl);
        } else if (header.msg_type == protocol_15m::kMsgTypeShipStatus) {
            protocol_15m::ShipStatus ship_status{};
            protocol_15m::DecodeResult result =
                protocol_15m::decodeShipStatus(receive_buffer, static_cast<size_t>(n), ship_status);
            if (!result.ok) {
                AERROR << "15m ship status invalid: " << result.error;
                continue;
            }
            AINFO << "15m ship status decoded: speed=" << ship_status.speed
                  << ", roll=" << ship_status.roll
                  << ", pitch=" << ship_status.pitch
                  << ", rudder=" << ship_status.rudder;
        } else {
            AERROR << "Unsupported 15m TCP message type: " << header.msg_type;
        }
    }
    close(client_fd);
    connections.erase(client_fd);
}

void TcpServer::returnTcpData(const Server_Info &info) {
    AINFO << "return bus";
    size_t len = packServerInfo(info, send_buffer);
    if (len == 0) {
        AERROR << "pack 15m interceptor status failed";
        return;
    }

    for (auto& fd : connections) {
        ssize_t sent = send(fd, send_buffer, len, 0);
        if (sent < 0) {
            AERROR << "send 15m interceptor status failed: " << strerror(errno);
        }
    }
}

protocol_15m::DecodeResult TcpServer::parseServerCtrl(const uint8_t* buf, size_t len, Server_Ctrl& out) {
    protocol_15m::ControlCommand command{};
    protocol_15m::DecodeResult result = protocol_15m::decodeControlCommand(buf, len, command);
    if (!result.ok) {
        return result;
    }

    out = toServerCtrl(command);
    return protocol_15m::DecodeResult::success();
}

size_t TcpServer::packServerInfo(const Server_Info& server_info, uint8_t* buffer) {
    protocol_15m::InterceptorStatus status = toInterceptorStatus(server_info);
    return protocol_15m::encodeInterceptorStatus(status, buffer, TCP_MAX_BUFFER_LENGTH);
}
