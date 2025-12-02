#pragma once
#include <thread>
#include <atomic>
#include <modbus/modbus.h>
#include "event_bus.h"
#include "data_struct.h"
#include <unistd.h>

class ModbusServer {
public:
    ModbusServer(EventBus& bus);
    ~ModbusServer();

    bool start(int port);
    void stop();

private:
    void serverLoop();
    void returnModbusData(const ModbusDataEvent& event);
    void changeByteOrder(uint8_t* data, size_t len);
    void changeWordOrder(uint8_t* data, size_t len);

    EventBus& bus_;
    std::atomic<bool> running_{false};
    std::thread serverThread_;
    modbus_t* ctx_ = nullptr;
    modbus_mapping_t* mapping_ = nullptr;
    int listen_fd_;

    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t reply[MODBUS_TCP_MAX_ADU_LENGTH];
};
