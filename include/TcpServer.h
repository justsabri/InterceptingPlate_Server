#pragma once
#include <thread>
#include <atomic>
#include <arpa/inet.h>
#include "event_bus.h"
#include "data_struct.h"
#include <unistd.h>
#include <set>

#define TCP_MAX_BUFFER_LENGTH 1024

class TcpServer {
public:
    TcpServer(EventBus& bus);
    ~TcpServer();

    bool start(int port);
    void stop();

private:
    void serverLoop();
    void handleClient(int client_fd);
    bool parseServerCtrl(const uint8_t* buf, size_t len, Server_Ctrl& out);
    size_t packServerInfo(const Server_Info& server_info, uint8_t* buffer);
    void returnTcpData(const Server_Info& info);

    EventBus& bus_;
    std::atomic<bool> running_{false};
    std::thread serverThread_;
    int server_fd = -1;
    int client_fd = -1;
    std::set<int> connections;
    // uint8_t receive_buffer[TCP_MAX_BUFFER_LENGTH];
    uint8_t send_buffer[TCP_MAX_BUFFER_LENGTH];
};
