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
#include "log.h"
#include <mutex>
using namespace std;

class ModbusRTUServer {
public:
    ModbusRTUServer() : running_(false) {
        AINFO<<"modbus_bus addr"<<&modbus_bus_;
        modbus_bus_.subscribe<ModbusData>("to_modbus_rtu", [this](const ModbusData& data) {
            AINFO<<"Modbus RTU服务器收到数据更新请求";
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
            AINFO << "无法初始化Modbus RTU上下文: " << modbus_strerror(errno);
            return false;
        }

        // 设置从站地址
        modbus_set_slave(ctx_, 1);

        // 连接到串口
        if (modbus_connect(ctx_) == -1) {
            AINFO << "无法连接到串口: " << modbus_strerror(errno);
            modbus_free(ctx_);
            return false;
        }

        // 创建modbus映射
        mapping_ = modbus_mapping_new(0, 0, 0x2030, 0x2030);
        if (!mapping_) {
            AINFO << "无法创建modbus映射: " << modbus_strerror(errno);
            modbus_free(ctx_);
            return false;
        }
        AINFO << "Modbus RTU服务器初始化成功，开始启动线程...";
        running_ = true;
        serverThread_ = std::thread([this]{ serverLoop(); });
        serverThread_.join();
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
    ModbusData modbusReceiveData_;
    ModbusData modbusSendData_;
    std::map<int, RegisterMap> registerMap_;
    modbus_t *ctx_ = nullptr;
    modbus_mapping_t *mapping_ = nullptr;
    std::thread serverThread_;
    std::atomic<bool> running_;
    uint8_t query_[MODBUS_TCP_MAX_ADU_LENGTH];
    // 初始化寄存器映射表
    void initRegisterMap() {
        // 控制参数寄存器（可写）
        registerMap_[1001] = {1001, 2, TYPE_INT16, &modbusSendData_.controlMode, true, 1, 2};
        registerMap_[1002] = {1002, 4, TYPE_FLOAT, &modbusSendData_.manualLeftExtend, true, 0.0f, 1.0f};
        registerMap_[1004] = {1004, 4, TYPE_FLOAT, &modbusSendData_.manualRightExtend, true, 0.0f, 1.0f};
        registerMap_[1006] = {1006, 2, TYPE_INT16, &modbusSendData_.autoModeParam, true, 1, 35};
        registerMap_[1007] = {1007, 4, TYPE_FLOAT, &modbusSendData_.rollAngle, true, -40.0f, 40.0f};
        registerMap_[1009] = {1009, 4, TYPE_FLOAT, &modbusSendData_.pitchAngle, true, -30.0f, 30.0f};
        registerMap_[1011] = {1011, 4, TYPE_FLOAT, &modbusSendData_.rudderAngle, true, -30.0f, 30.0f};
        registerMap_[1013] = {1013, 4, TYPE_FLOAT, &modbusSendData_.speed, true, 0.0f, 80.0f};
        registerMap_[1015] = {1015, 4, TYPE_FLOAT, &modbusSendData_.timestamp, true, 0.0f, 2147483647.0f};
        registerMap_[1017] = {1017, 8, TYPE_DOUBLE, &modbusSendData_.longitude, true, -180.0f, 180.0f};
        registerMap_[1021] = {1021, 8, TYPE_DOUBLE, &modbusSendData_.latitude, true, -90.0f, 90.0f};
        registerMap_[1025] = {1025, 4, TYPE_FLOAT, &modbusSendData_.leftEngineSpeed, true, 0.0f, 5000.0f};
        registerMap_[1027] = {1027, 4, TYPE_FLOAT, &modbusSendData_.rightEngineSpeed, true, 0.0f, 5000.0f};
        registerMap_[1029] = {1029, 2, TYPE_INT16, &modbusSendData_.leftEngineGear, true, 0, 3};
        registerMap_[1030] = {1030, 2, TYPE_INT16, &modbusSendData_.rightEngineGear, true, 0, 3};

        // 状态数据寄存器（只读）
        registerMap_[2001] = {2001, 4, TYPE_FLOAT, &modbusReceiveData_.currentSpeed, false, 0.0f, 100.0f};
        registerMap_[2003] = {2003, 4, TYPE_FLOAT, &modbusReceiveData_.leftExtendThreshold, false, 0.0f, 1.0f};
        registerMap_[2005] = {2005, 4, TYPE_FLOAT, &modbusReceiveData_.rightExtendThreshold, false, 0.0f, 1.0f};
        registerMap_[2007] = {2007, 4, TYPE_FLOAT, &modbusReceiveData_.leftCurrentExtend, false, 0.0f, 1.0f};
        registerMap_[2009] = {2009, 4, TYPE_FLOAT, &modbusReceiveData_.rightCurrentExtend, false, 0.0f, 1.0f};
        registerMap_[2011] = {2011, 2, TYPE_INT16, &modbusReceiveData_.motorCount, false, 0, 4};
        registerMap_[2012] = {2012, 2, TYPE_INT16, &modbusReceiveData_.motor1Status, false, 0, 999};
        registerMap_[2013] = {2013, 2, TYPE_INT16, &modbusReceiveData_.motor2Status, false, 0, 999};
        registerMap_[2014] = {2014, 2, TYPE_INT16, &modbusReceiveData_.motor3Status, false, 0, 999};
        registerMap_[2015] = {2015, 2, TYPE_INT16, &modbusReceiveData_.motor4Status, false, 0, 999};
        registerMap_[2016] = {2016, 2, TYPE_INT16, &modbusReceiveData_.imuStatus, false, 0, 999};
        registerMap_[2017] = {2017, 2, TYPE_INT16, &modbusReceiveData_.slaveStatus, false, 0, 999};
        registerMap_[2018] = {2018, 4, TYPE_FLOAT, &modbusReceiveData_.currentPitch, false, -30.0f, 30.0f};
        registerMap_[2020] = {2020, 4, TYPE_FLOAT, &modbusReceiveData_.currentRoll, false, -40.0f, 40.0f};
    }

    // 处理保持寄存器读取
    void handleReadHoldingRegisters(int startAddr, int nb) {
        AINFO << "处理保持寄存器读取请求: 起始地址=" << startAddr << ", 数量=" << nb;
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
                            {
                                // 正确处理大端序浮点数
                                float value = *(float *)reg.dataPtr;
                                uint32_t raw_value;
                                memcpy(&raw_value, &value, sizeof(float));
                                mapping_->tab_registers[currentAddr] = (raw_value >> 16) & 0xFFFF;
                                mapping_->tab_registers[currentAddr + 1] = raw_value & 0xFFFF;
                            }
                            break;
                        case TYPE_DOUBLE:
                            {
                                // 正确处理大端序双精度浮点数
                                double value = *(double *)reg.dataPtr;
                                uint64_t raw_value;
                                memcpy(&raw_value, &value, sizeof(double));
                                mapping_->tab_registers[currentAddr] = (raw_value >> 48) & 0xFFFF;
                                mapping_->tab_registers[currentAddr + 1] = (raw_value >> 32) & 0xFFFF;
                                mapping_->tab_registers[currentAddr + 2] = (raw_value >> 16) & 0xFFFF;
                                mapping_->tab_registers[currentAddr + 3] = raw_value & 0xFFFF;
                            }
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
        AINFO << "保持寄存器读取完成: 起始地址=" << startAddr << ", 数量=" << nb;
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
                                // 正确处理大端序浮点数
                                uint32_t raw_value = (data[dataIndex] << 16) | data[dataIndex + 1];
                                float value;
                                memcpy(&value, &raw_value, sizeof(float));
                                if (value >= reg.minValue && value <= reg.maxValue) {
                                    *(float *)reg.dataPtr = value;
                                }
                            }
                            break;
                        case TYPE_DOUBLE:
                            {
                                // 正确处理大端序双精度浮点数
                                uint64_t raw_value = ((uint64_t)data[dataIndex] << 48) | 
                                                    ((uint64_t)data[dataIndex + 1] << 32) | 
                                                    ((uint64_t)data[dataIndex + 2] << 16) | 
                                                    data[dataIndex + 3];
                                double value;
                                memcpy(&value, &raw_value, sizeof(double));
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


    void updateModbusData(const ModbusData& data) {
        memcpy(&modbusReceiveData_, &data, sizeof(struct ModbusData));
    }
    // 服务器循环
    void serverLoop() {
        AINFO << "Modbus RTU服务器线程已启动，等待请求...";
        while (running_) {
            AINFO << "modbusrtu循环开始";
            int rc = modbus_receive(ctx_, query_);
            AINFO << "modbus_receive返回: " << rc;
            if (rc > 0) {
                // 打印接收到的完整报文
                AINFO << "收到Modbus RTU报文: 长度=" << rc << ", 内容=";
                for (int i = 0; i < rc; i++) {
                    printf("%02X ", query_[i]);
                }
                printf("\n");
                
                uint8_t func_code = query_[1];
                AINFO << "收到请求: 功能码=" << (int)func_code;
                switch (func_code) {
                    case MODBUS_FC_READ_HOLDING_REGISTERS:
                        {
                            uint16_t addr = query_[2] << 8 | query_[3];
                            uint16_t count = query_[4] << 8 | query_[5];
                            AINFO << "读取保持寄存器: 地址=" << addr << ", 数量=" << count;
                            modbusReceiveData_.dataFlow = MODBUS_FC_READ_HOLDING_REGISTERS;
                            modbus_bus_.publish("from_modbus_rtu", modbusReceiveData_);
                            handleReadHoldingRegisters(addr, count);
                            // 打印发送的响应
                            AINFO << "发送Modbus RTU响应: 功能码=0x03, 地址=" << addr << ", 数量=" << count;
                            modbus_reply(ctx_, query_, rc, mapping_);
                            break;
                        }

                    case MODBUS_FC_WRITE_SINGLE_REGISTER:
                        {
                            uint16_t addr = query_[2] << 8 | query_[3];
                            uint16_t value = query_[4] << 8 | query_[5];
                            AINFO << "写入单个寄存器: 地址=" << addr << ", 值=" << value;
                            modbusSendData_.dataFlow = MODBUS_FC_WRITE_SINGLE_REGISTER;
                            handleWriteMultipleRegisters(addr, 1, &value);
                            // 打印发送的响应
                            AINFO << "发送Modbus RTU响应: 功能码=0x06, 地址=" << addr << ", 值=" << value;
                            modbus_reply(ctx_, query_, rc, mapping_);
                            break;
                        }

                    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                        {
                            uint16_t addr = query_[2] << 8 | query_[3];
                            uint16_t count = query_[4] << 8 | query_[5];
                            AINFO << "写入多个寄存器: 地址=" << addr << ", 数量=" << count;
                            modbusSendData_.dataFlow = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
                            int data_start = 7;
                            uint16_t data[count];
                            AINFO << "写入数据: ";
                            for (int i = 0; i < count; i++) {
                                data[i] = (query_[data_start + 2 * i] << 8) | query_[data_start + 2 * i + 1];
                                printf("%d (0x%04X) ", data[i], data[i]);
                            }
                            printf("\n");
                            handleWriteMultipleRegisters(addr, count, data);
                            modbus_bus_.publish("from_modbus_rtu", modbusSendData_);
                            modbus_bus_.publish("to_imu", modbusSendData_);
                            AINFO<<"timestamp"<<modbusSendData_.timestamp;
                            // 打印发送的响应
                            AINFO << "发送Modbus RTU响应: 功能码=0x10, 地址=" << addr << ", 数量=" << count;
                            modbus_reply(ctx_, query_, rc, mapping_);
                            break;
                        }

                    default:
                        printf("未知功能码: %d\n", func_code);
                        break;
                }
            } else if (rc == -1) {
                AINFO<< "接收请求失败: " << modbus_strerror(errno);

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
