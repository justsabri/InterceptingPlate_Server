#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <modbus/modbus.h>
#include <map>
#include <thread>
#include <atomic>
#include "data_struct.h"
#include "event_bus.h"
#include "modbus_rtu_bus.h"

using namespace std;

class ModbusRTUServer {
public:
    ModbusRTUServer() : running_(false) {
        modbus_bus_.subscribe<ModbusData>("to_modbus_rtu", [this](const ModbusData data) {
            updateModbusData(data);
        });
    }

    ~ModbusRTUServer() { stop(); }

    bool start(const std::string& port, int baud, char parity, int dataBits, int stopBits) {
        // 初始化寄存器映射表
        initRegisterMap();

        // 初始化Modbus上下文
        ctx_ = modbus_new_rtu(port.c_str(), baud, parity, dataBits, stopBits);
        if (!ctx_) {
            fprintf(stderr, "无法初始化Modbus RTU上下文: %s\n", modbus_strerror(errno));
            return false;
        }

        // 设置从站地址
        modbus_set_slave(ctx_, 1);

        // 连接到串口
        if (modbus_connect(ctx_) == -1) {
            fprintf(stderr, "无法连接到串口: %s\n", modbus_strerror(errno));
            modbus_free(ctx_);
            return false;
        }

        // 创建modbus映射
        mapping_ = modbus_mapping_new(0, 0, 0x2030, 0x2030);
        if (!mapping_) {
            fprintf(stderr, "无法创建modbus映射: %s\n", modbus_strerror(errno));
            modbus_free(ctx_);
            return false;
        }

        running_ = true;
        serverThread_ = std::thread([this]{ serverLoop(); });
        return true;
    }

    void stop() {
        running_ = false;
        if (serverThread_.joinable()) {
            serverThread_.join();
        }
        if (mapping_) {
            modbus_mapping_free(mapping_);
            mapping_ = nullptr;
        }
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
    }

private:
    ModbusData modbusData_;
    std::map<int, RegisterMap> registerMap_;
    modbus_t *ctx_ = nullptr;
    modbus_mapping_t *mapping_ = nullptr;
    std::thread serverThread_;
    std::atomic<bool> running_;
    uint8_t query_[MODBUS_TCP_MAX_ADU_LENGTH];

    // 初始化寄存器映射表
    void initRegisterMap() {
        // 控制参数寄存器（可写）
        registerMap_[1001] = {1001, 2, TYPE_INT16, &modbusData_.controlMode, true, 1, 2};
        registerMap_[1002] = {1002, 4, TYPE_FLOAT, &modbusData_.manualLeftExtend, true, 0.0f, 1.0f};
        registerMap_[1004] = {1004, 4, TYPE_FLOAT, &modbusData_.manualRightExtend, true, 0.0f, 1.0f};
        registerMap_[1006] = {1006, 2, TYPE_INT16, &modbusData_.autoModeParam, true, 1, 35};
        registerMap_[1007] = {1007, 4, TYPE_FLOAT, &modbusData_.rollAngle, true, -40.0f, 40.0f};
        registerMap_[1009] = {1009, 4, TYPE_FLOAT, &modbusData_.pitchAngle, true, -30.0f, 30.0f};
        registerMap_[1011] = {1011, 4, TYPE_FLOAT, &modbusData_.rudderAngle, true, -30.0f, 30.0f};
        registerMap_[1013] = {1013, 4, TYPE_FLOAT, &modbusData_.speed, true, 0.0f, 80.0f};
        registerMap_[1015] = {1015, 4, TYPE_FLOAT, &modbusData_.timestamp, true, 0.0f, 999999999.0f};
        registerMap_[1017] = {1017, 8, TYPE_DOUBLE, &modbusData_.longitude, true, -180.0f, 180.0f};
        registerMap_[1021] = {1021, 8, TYPE_DOUBLE, &modbusData_.latitude, true, -90.0f, 90.0f};
        registerMap_[1025] = {1025, 4, TYPE_FLOAT, &modbusData_.leftEngineSpeed, true, 0.0f, 5000.0f};
        registerMap_[1027] = {1027, 4, TYPE_FLOAT, &modbusData_.rightEngineSpeed, true, 0.0f, 5000.0f};
        registerMap_[1029] = {1029, 2, TYPE_INT16, &modbusData_.leftEngineGear, true, 0, 3};
        registerMap_[1030] = {1030, 2, TYPE_INT16, &modbusData_.rightEngineGear, true, 0, 3};

        // 状态数据寄存器（只读）
        registerMap_[2001] = {2001, 4, TYPE_FLOAT, &modbusData_.currentSpeed, false, 0.0f, 100.0f};
        registerMap_[2003] = {2003, 4, TYPE_FLOAT, &modbusData_.leftExtendThreshold, false, 0.0f, 1.0f};
        registerMap_[2005] = {2005, 4, TYPE_FLOAT, &modbusData_.rightExtendThreshold, false, 0.0f, 1.0f};
        registerMap_[2007] = {2007, 4, TYPE_FLOAT, &modbusData_.leftCurrentExtend, false, 0.0f, 1.0f};
        registerMap_[2009] = {2009, 4, TYPE_FLOAT, &modbusData_.rightCurrentExtend, false, 0.0f, 1.0f};
        registerMap_[2011] = {2011, 2, TYPE_INT16, &modbusData_.motorCount, false, 0, 4};
        registerMap_[2012] = {2012, 2, TYPE_INT16, &modbusData_.motor1Status, false, 0, 999};
        registerMap_[2013] = {2013, 2, TYPE_INT16, &modbusData_.motor2Status, false, 0, 999};
        registerMap_[2014] = {2014, 2, TYPE_INT16, &modbusData_.motor3Status, false, 0, 999};
        registerMap_[2015] = {2015, 2, TYPE_INT16, &modbusData_.motor4Status, false, 0, 999};
        registerMap_[2016] = {2016, 2, TYPE_INT16, &modbusData_.imuStatus, false, 0, 999};
        registerMap_[2017] = {2017, 2, TYPE_INT16, &modbusData_.slaveStatus, false, 0, 999};
        registerMap_[2020] = {2020, 4, TYPE_FLOAT, &modbusData_.currentPitch, false, -30.0f, 30.0f};
        registerMap_[2022] = {2022, 4, TYPE_FLOAT, &modbusData_.currentRoll, false, -40.0f, 40.0f};
    }

    // 处理保持寄存器读取
    void handleReadHoldingRegisters(int startAddr, int nb) {
        int currentAddr = startAddr;
        int remaining = nb;
        
        while (remaining > 0) {
            auto it = registerMap_.find(currentAddr);
            if (it != registerMap_.end()) {
                RegisterMap &reg = it->second;
                int regCount = reg.length / 2; // 寄存器数量（每个寄存器2字节）
                
                if (remaining >= regCount) {
                    switch (reg.type) {
                        case TYPE_INT16:
                            mapping_->tab_registers[currentAddr] = *(int16_t *)reg.dataPtr;
                            break;
                        case TYPE_FLOAT:
                            memcpy(&mapping_->tab_registers[currentAddr], reg.dataPtr, sizeof(float));
                            break;
                        case TYPE_DOUBLE:
                            memcpy(&mapping_->tab_registers[currentAddr], reg.dataPtr, sizeof(double));
                            break;
                    }
                    
                    // 移动到下一个寄存器
                    currentAddr += regCount;
                    remaining -= regCount;
                } else {
                    // 寄存器长度大于剩余数量，跳过
                    currentAddr++;
                    remaining--;
                }
            } else {
                // 寄存器不存在，跳过
                currentAddr++;
                remaining--;
            }
        }
    }

    // 处理保持寄存器写入
    void handleWriteMultipleRegisters(int startAddr, int nb, const uint16_t *data) {
        int currentAddr = startAddr;
        int dataIndex = 0;
        
        while (nb > 0) {
            auto it = registerMap_.find(currentAddr);
            if (it != registerMap_.end()) {
                RegisterMap &reg = it->second;
                int regCount = reg.length / 2; // 寄存器数量（每个寄存器2字节）
                
                if (reg.writable && nb >= regCount) {
                    switch (reg.type) {
                        case TYPE_INT16:
                            {
                                int16_t value = data[dataIndex];
                                if (value >= reg.minValue && value <= reg.maxValue) {
                                    *(int16_t *)reg.dataPtr = value;
                                }
                            }
                            break;
                        case TYPE_FLOAT:
                            {
                                float value;
                                memcpy(&value, &data[dataIndex], sizeof(float));
                                if (value >= reg.minValue && value <= reg.maxValue) {
                                    *(float *)reg.dataPtr = value;
                                }
                            }
                            break;
                        case TYPE_DOUBLE:
                            {
                                double value;
                                memcpy(&value, &data[dataIndex], sizeof(double));
                                if (value >= reg.minValue && value <= reg.maxValue) {
                                    *(double *)reg.dataPtr = value;
                                }
                            }
                            break;
                    }
                    
                    // 移动到下一个寄存器
                    currentAddr += regCount;
                    dataIndex += regCount;
                    nb -= regCount;
                } else {
                    // 无法处理当前寄存器，跳过
                    currentAddr++;
                    dataIndex++;
                    nb--;
                }
            } else {
                // 寄存器不存在，跳过
                currentAddr++;
                dataIndex++;
                nb--;
            }
        }
    }

    void updateModbusData(ModbusData data) {
        memcpy(&modbusData_, &data, sizeof(struct ModbusData));
    }
    // 服务器循环
    void serverLoop() {
        printf("Modbus RTU从站已启动，等待请求...\n");

        while (running_) {
            int rc = modbus_receive(ctx_, query_);
            if (rc > 0) {
                uint8_t func_code = query_[1];
                printf("收到请求: 功能码=%d\n", func_code);

                switch (func_code) {
                    case MODBUS_FC_READ_HOLDING_REGISTERS:
                        {
                            uint16_t addr = query_[2] << 8 | query_[3];
                            uint16_t count = query_[4] << 8 | query_[5];
                            printf("读取保持寄存器: 地址=%d, 数量=%d\n", addr, count);
                            modbusData_.dataFlow = MODBUS_FC_READ_HOLDING_REGISTERS;
                            modbus_bus_.publish("from_modbus_rtu", &modbusData_);
                            handleReadHoldingRegisters(addr, count);
                            modbus_reply(ctx_, query_, rc, mapping_);
                            break;
                        }

                    case MODBUS_FC_WRITE_SINGLE_REGISTER:
                        {
                            uint16_t addr = query_[2] << 8 | query_[3];
                            uint16_t value = query_[4] << 8 | query_[5];
                            printf("写入单个寄存器: 地址=%d, 值=%d\n", addr, value);
                            modbusData_.dataFlow = MODBUS_FC_WRITE_SINGLE_REGISTER;
                            handleWriteMultipleRegisters(addr, 1, &value);
                            modbus_reply(ctx_, query_, rc, mapping_);
                            break;
                        }

                    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                        {
                            uint16_t addr = query_[2] << 8 | query_[3];
                            uint16_t count = query_[4] << 8 | query_[5];
                            printf("写入多个寄存器: 地址=%d, 数量=%d\n", addr, count);
                            modbusData_.dataFlow = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
                            int data_start = 7;
                            uint16_t data[count];
                            for (int i = 0; i < count; i++) {
                                data[i] = (query_[data_start + 2 * i] << 8) | query_[data_start + 2 * i + 1];
                            }
                            handleWriteMultipleRegisters(addr, count, data);
                            modbus_bus_.publish("from_modbus_rtu", modbusData_);
                            modbus_bus_.publish("to_imu", modbusData_);
                            modbus_reply(ctx_, query_, rc, mapping_);
                            break;
                        }

                    default:
                        printf("未知功能码: %d\n", func_code);
                        break;
                }
            } else if (rc == -1) {
                fprintf(stderr, "接收请求失败: %s\n", modbus_strerror(errno));
                // 继续循环，等待下一个请求
            }
        }
    }
};

// int main() {
//     ModbusRTUServer server;
    
//     // 启动服务器
//     if (!server.start("/dev/ttyUSB0", 115200, 'N', 8, 1)) {
//         fprintf(stderr, "无法启动Modbus RTU服务器\n");
//         return -1;
//     }

//     printf("Modbus RTU服务器启动成功\n");
//     printf("按任意键停止服务器...\n");
//     getchar();

//     // 停止服务器
//     server.stop();
//     printf("Modbus RTU服务器已停止\n");

//     return 0;
// }
