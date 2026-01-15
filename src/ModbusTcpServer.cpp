#include "ModbusTcpServer.h"
#include <iostream>
#include <cstring>
#include <log.h>

ModbusServer::ModbusServer(EventBus& bus) : bus_(bus) {
    // 订阅控制命令事件
    bus_.subscribe<ModbusDataEvent>("to_modbus",
        [this](const ModbusDataEvent& event){ returnModbusData(event); });
}

ModbusServer::~ModbusServer() {
    stop();
}

bool ModbusServer::start(int port) {
    ctx_ = modbus_new_tcp("192.168.0.200", port);
    if (!ctx_) {
        AINFO << "Failed to create modbus context\n";
        return false;
    }

    mapping_ = modbus_mapping_new(0, 0, 0x2030, 0x2030);
    if (!mapping_) {
        AINFO << "Failed to allocate modbus mapping\n";
        modbus_free(ctx_);
        return false;
    }

    listen_fd_ = modbus_tcp_listen(ctx_, 1);
    if (listen_fd_ < 0) {
        AINFO << "Listen failed\n";
        return false;
    }

    running_ = true;
    serverThread_ = std::thread([this]{ serverLoop(); });
    serverThread_.join();
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
        AINFO << "Waiting for new client..." << std::endl;
        int client_socket = modbus_tcp_accept(ctx_, &listen_fd_);
        if (client_socket < 0) {
            perror("modbus_tcp_accept failed");
            continue;
        }
        AINFO << "Client connected." << std::endl;

        while (running_) {
            int rc = modbus_receive(ctx_, query);
            if (rc > 0) {
                uint8_t func_code = query[7];
                AINFO << "modbus " << std::hex << static_cast<int>(func_code);
                switch (func_code) {
                    case 0x03:
                        {
                            uint16_t addr = query[8] << 8 | query[9];
                            uint16_t count = query[10] << 8 | query[11];
                            ModbusDataEvent event{"GET", addr, count, count*2, (uint8_t*)&mapping_->tab_registers[addr]};
                            bus_.publish("from_modbus", event);
                            uint16_t num = event.frame[1] << 8 | event.frame[0];
                            AINFO << "reply " << static_cast<int>(num);

                            AINFO << "nb_registers=" << mapping_->nb_registers;
                            if (addr + count > mapping_->nb_registers) {
                                AINFO << "请求超出范围，modbus_reply 会返回异常";
                            }
                            // changeWordOrder((uint8_t*)&mapping_->tab_registers[addr], count*2);
                            modbus_reply(ctx_, query, rc, mapping_);
                            break;
                        }

                    case 0x06:
                        {
                            uint16_t addr = query[8] << 8 | query[9];
                            uint16_t count = 1;
                            ModbusDataEvent event{"POST", addr, count, count*2, (uint8_t*)&mapping_->tab_registers[addr]};
                            modbus_reply(ctx_, query, rc, mapping_);
                            bus_.publish("from_modbus", event);
                            break;
                        }

                    case 0x10:
                        {
                            uint16_t addr = query[8] << 8 | query[9];
                            uint16_t count = query[10] << 8 | query[11];
                            ModbusDataEvent event{"POST", addr, count, count*2, (uint8_t*)&mapping_->tab_registers[addr]};
                            modbus_reply(ctx_, query, rc, mapping_);
                            bus_.publish("from_modbus", event);
                            break;
                        }

                    default:
                        break;
                }
            } else if (rc == -1) {
                perror("modbus_receive error");
                AINFO << "Client disconnected.\n";
                break; // 跳出内层循环，重新accept新客户端
            }
        }
    }

    close(listen_fd_);
}

void ModbusServer::returnModbusData(const ModbusDataEvent &event) {
    AINFO << "return bus";
}

void ModbusServer::changeByteOrder(uint8_t* data, size_t len) {
    // 对每两个字节进行交换
    for (size_t i = 0; i + 1 < len; i += 2)
    {
        uint8_t tmp = data[i];
        data[i] = data[i + 1];
        data[i + 1] = tmp;
    }
}

void ModbusServer::changeWordOrder(uint8_t* data, size_t len) {
    if (len <= 2)
        return;

    size_t wordCount = len / 2;

    for (size_t i = 0; i < wordCount / 2; ++i) {
        size_t left  = i * 2;
        size_t right = (wordCount - 1 - i) * 2;

        // 交换两个 word（每个 word 2 字节）
        uint8_t tmp0 = data[left];
        uint8_t tmp1 = data[left + 1];

        data[left]     = data[right];
        data[left + 1] = data[right + 1];

        data[right]     = tmp0;
        data[right + 1] = tmp1;
    }
}
