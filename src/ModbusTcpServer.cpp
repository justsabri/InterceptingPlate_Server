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
    ctx_ = modbus_new_tcp("0.0.0.0", port);
    if (!ctx_) {
        std::cerr << "Failed to create modbus context\n";
        return false;
    }

    mapping_ = modbus_mapping_new(0, 0, 100, 100);
    if (!mapping_) {
        std::cerr << "Failed to allocate modbus mapping\n";
        modbus_free(ctx_);
        return false;
    }

    int socket = modbus_tcp_listen(ctx_, 1);
    if (socket < 0) {
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
    int server_socket = modbus_tcp_listen(ctx_, 1);
    modbus_tcp_accept(ctx_, &server_socket);

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
            std::cerr << "Client disconnected\n";
            modbus_tcp_accept(ctx_, &server_socket);
        }
    }
}

void ModbusServer::returnModbusData(const ModbusDataEvent &event) {

}


