#include "ModbusTcpServer.h"
#include <iostream>
#include <cstring>

ModbusServer::ModbusServer(EventBus& bus) : bus_(bus) {
    // 订阅控制命令事件
    bus_.subscribe<ModbusDataEvent>("to_modbus",
        [this](const ModbusDataEvent& event){ returnModbusData(event); });
}

ModbusServer::~ModbusServer() {
    stop();
}

bool ModbusServer::start(int port) {
    ctx_ = modbus_new_tcp("192.168.1.77", port);
    if (!ctx_) {
        std::cerr << "Failed to create modbus context\n";
        return false;
    }

    mapping_ = modbus_mapping_new(0, 0, 2500, 2500);
    if (!mapping_) {
        std::cerr << "Failed to allocate modbus mapping\n";
        modbus_free(ctx_);
        return false;
    }

    listen_fd_ = modbus_tcp_listen(ctx_, 1);
    if (listen_fd_ < 0) {
        std::cerr << "Listen failed\n";
        return false;
    }

    running_ = true;
    serverThread_ = std::thread([this]{ serverLoop(); });
    return true;
}

void ModbusServer::stop() {
    running_ = false;
    if (serverThread_.joinable()) serverThread_.join();
    if (mapping_) modbus_mapping_free(mapping_);
    if (ctx_) modbus_free(ctx_);
}

void ModbusServer::serverLoop() {
    // int server_socket = modbus_tcp_listen(ctx_, 1);
    // modbus_tcp_accept(ctx_, &listen_fd_);

    while (running_) {
        std::cout << "Waiting for new client..." << std::endl;
        int client_socket = modbus_tcp_accept(ctx_, &listen_fd_);
        if (client_socket < 0) {
            perror("modbus_tcp_accept failed");
            continue;
        }
        std::cout << "Client connected." << std::endl;

        while (running_) {
            int rc = modbus_receive(ctx_, query);
            if (rc > 0) {
                uint8_t func_code = query[7];
                switch (func_code) {
                    case 0x03:
                        {
                            uint16_t addr = query[8] << 8 | query[9];
                            uint16_t count = query[10] << 8 | query[11];
                            ModbusDataEvent event{"GET", addr, count, count*2, (uint8_t*)&mapping_->tab_registers[addr]};
                            bus_.publish("from_modbus", event);
                            modbus_reply(ctx_, query, rc, mapping_);
                            break;
                        }

                    case 0x06:
                        {
                            uint16_t addr = query[8] << 8 | query[9];
                            uint16_t count = 1;
                            ModbusDataEvent event{"POST", addr, count, count*2, (uint8_t*)&mapping_->tab_registers[addr]};
                            bus_.publish("from_modbus", event);
                            modbus_reply(ctx_, query, rc, mapping_);
                            break;
                        }

                    case 0x10:
                        {
                            uint16_t addr = query[8] << 8 | query[9];
                            uint16_t count = query[10] << 8 | query[11];
                            ModbusDataEvent event{"POST", addr, count, count*2, (uint8_t*)&mapping_->tab_registers[addr]};
                            bus_.publish("from_modbus", event);
                            modbus_reply(ctx_, query, rc, mapping_);
                            break;
                        }

                    default:
                        break;
                }
            } else if (rc == -1) {
                perror("modbus_receive error");
                std::cerr << "Client disconnected.\n";
                break; // 跳出内层循环，重新accept新客户端
            }
        }
    }

    close(listen_fd_);
}

void ModbusServer::returnModbusData(const ModbusDataEvent &event) {

}


