#pragma once
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <cstdint>
#include "event_bus.h"
// // 传感器数据结构体
// struct ImuData
// {
//   uint8_t imu_status_key = 0; // IMU状态字    bit0:1 GNSS数据有效 bit1:1 PPS信号有效 bit2:1 初始化有效
//                               //             bit4-3: 11 GNSS板卡初始化成功
//   uint8_t imu_work_status = 0; // IMU 工作状态 1:对准  2：陀螺零位标定  7：纯惯性导航  9：与GNSS组合导航
//   // GPS数据
//   uint16_t gps_week = 0;        // GPS周计数
//   uint32_t gps_millisecond = 0; // GPS周内秒
//   uint8_t GNSS_staus = 0;       // GNSS定位状态 0：不可用或无效  1：单点定位  2：差分定位  3：GPS PPS模式
//                           // 4：RTK固定解 5：RTK浮点解 6：惯导模式 7：手动输入模式  8：模拟器模式
//   bool disconnect = false;
//   // 姿态数据
//   double yaw = 2.5;           // 艏向角
//   double pitch = 0.0;         // 纵倾角
//   double roll = 0.0;          // 横倾角
//   uint8_t satellite_num = 0;  // 卫星数量
//   uint8_t posture_status = 0; // 姿态有效位 0：无效  1：有效

//   // 位置数据
//   double latitude = 0.0;  // 经度
//   double longitude = 0.0; // 纬度

//   // 速度数据
//   double altitude = 0.0;       // 海拔
//   double north_velocity = 0.0; // 北向速度
//   double east_velocity = 0.0;  // 东向速度
//   double down_velocity = 0.0;  // 地向速度

//   // 陀螺数据
//   double gyro_x = 0.0;  // 陀螺仪x轴
//   double gyro_y = 0.0;  // 陀螺仪y轴
//   double gyro_z = 0.0;  // 陀螺仪z轴
//   double heading = 0.0; // 双天线航向角

//   // 加速度与温度
//   double acc_x = 0.0;       // 加速度计x轴
//   double acc_y = 0.0;       // 加速度计y轴
//   double acc_z = 0.0;       // 加速度计z轴
//   double temperature = 0.0; // 温度
//   uint8_t antenna_type = 0; // 双天线航向角类型

//   //---------------------1、船舶当前舵角-------------------------------------
//   // 20250822 田鸿宇 新算法用到  船舶  舵角参数
//   double current_rudder;   // 船舶当前舵角
//   //---------------------1、船舶当前舵角-------------------------------------
// };

// 根据表格定义的外部信息系统数据结构
struct ImuData {
    float roll;          // 横倾角 (-40°~40°)
    float pitch;         // 纵倾角 (-30°~30°)
    float rudder;        // 舵角 (-30°~30°)
    float speed;         // 航速 (0~80kn)
    float rpm;           // 主机转速 (0~5000rpm)
    uint32_t timestamp;  // UNIX时间戳
    std::string gps_time;
    float heading;
    double latitude;
    double longitude;
    float yaw;
    float left_rpm; //左侧主机转速
    float right_rpm; //右侧主机转速
    int left_gear; //左侧主机挡位
    int right_gear; //右侧主机挡位
};

// 电机错误状态结构体
struct MotorErrorStatus {
    bool bit0_softwareError = false;
    bool bit1_overVoltage = false;
    bool bit2_underVoltage = false;
    bool bit4_startupError = false;
    bool bit5_speedFeedbackError = false;
    bool bit6_overCurrent = false;
    bool bit7_softwareErrorOther = false;
    bool bit16_encoderCommError = false;
    bool bit17_motorOverTemp = false;
    bool bit18_boardOverTemp = false;
    bool bit19_driverChipError = false;

    void print() const;

    bool has_error() {
        return bit0_softwareError | bit1_overVoltage | bit2_underVoltage | bit4_startupError | bit5_speedFeedbackError
            | bit6_overCurrent | bit7_softwareErrorOther | bit16_encoderCommError | bit17_motorOverTemp 
            | bit18_boardOverTemp | bit19_driverChipError;
    };
};

// 电机数据结构体
struct MotorData {
    MotorErrorStatus status;//电机错误状态
    std::string mode; //电机运行模式
    double voltage;   //电机电压
    double current;   //电机电流
    double position;  //电机当前位置
    int32_t position_offset;  //电机零位偏移
    double encoder_battery_voltage; //编码器电池电压
    double max_forward_speed;   //最大允许正向速度
    double min_reverse_speed;   //最小允许负向速度
    double max_forward_position;    //最大正向位置
    double min_reverse_position;    //最小负向位置
    double temperature; //电机温度
    bool disconnect = false; // 电机断开
};
//工控板参数结构体
struct LinuxPcData {
  double temperature;      // 温度(℃) [-20, 60]
  double voltage;          // 电压(V) [24, 28]
  double storage_usage;    // 内存使用率(%) [0, 100]
  double cpu_usage;        // CPU使用率(%) [0, 100]
};
//device init return status
struct InitDeviceStatus {
  int motor;
  int imu;
};
//电机状态
struct MotorStateData {
    float plate;  //当前截流板伸出量（电机转动角度）实际：0~100° 理论：0~113°
    uint16_t alarm_code; //电机状态码
};
// 惯导状态
struct ImuStateData
{
  uint16_t alarm_code;       // 惯导状态码
  float yaw;
  float pitch;          // 纵倾角
  float roll;           // 横倾角
  float speed;          // 航速
  float rpm;           // 主机转速 (0~5000rpm)
  double latitude;
  double longitude;
  std::string gps_time; // GPS时间（北京时间，年月日时分秒）
  //---------------------1、船舶当前舵角-------------------------------------
  // 20250822 田鸿宇 新算法用到  船舶  舵角参数
  // 船舶当前舵角
  float current_rudder;
  
  float left_rpm; //左侧主机转速
  float right_rpm; //右侧主机转速
  int left_gear; //左侧主机挡位
  int right_gear; //右侧主机挡位
};
//下位机状态
struct PCStateData {
    uint16_t alarm_code; //下位机状态码
};

//数据打包结构体
struct DataPack {
  std::map<int, MotorStateData> motor_state; //电机状态<编号，值>
  ImuStateData imu_state; //惯导状态
  PCStateData pc_state; //下位机状态
};

typedef enum {
    STATE_DATA,

} Data_Type;

struct ModbusDataEvent {
    std::string func;
    uint16_t addr;
    uint16_t count;
    int len;
    uint8_t* frame;
};

// typedef enum {
//     MODBUS_PARAM_TYPE_FLOAT,
//     MODBUS_PARAM_TYPE_INT32,
//     MODBUS_PARAM_TYPE_UINT16,
// } ModbusParamType;

typedef struct {
    const std::string name;
    // ModbusParamType type;
    void* pointer;
    uint16_t modbus_addr;
    void (*handler_ptr)(void* ptr);
} ModbusParamItem;

struct TcpDataEvent {
    int recv_len;
    int send_len;
    uint8_t* recv_buffer;
    uint8_t* send_buffer;
};

struct Server_Info {
    float speed;
    float ext_left_limit;
    float ext_right_limit;
    float ext_left;
    float ext_right;
    uint16_t motor_num;
    std::vector<uint16_t> motor_state;
    uint16_t pc_state;
    float heading;
    float pitch;
    float roll;
    uint64_t timestamp;
};

struct Server_Ctrl {
    uint16_t ctrl_mode;
    float ext_left;
    float ext_right;
    uint16_t shutdown;
    uint64_t timestamp;
};

// 定义数据类型枚举
enum DataType {
    TYPE_INT16,
    TYPE_FLOAT,
    TYPE_DOUBLE
};

// 定义寄存器映射结构体
struct RegisterMap {
    int address;           // 寄存器地址
    int length;            // 寄存器长度（字节数）
    DataType type;         // 数据类型
    void* dataPtr;         // 数据指针
    bool writable;         // 是否可写
    float minValue;        // 最小值
    float maxValue;        // 最大值
};

// 数据存储结构
struct ModbusData {
    // 数据方向
    int16_t dataFlow;
    // 控制参数（1001-1030）
    int16_t controlMode;           // 1001: 控制模式
    float manualLeftExtend;         // 1002-1003: 手动参数左侧伸出百分比
    float manualRightExtend;        // 1004-1005: 手动参数右侧伸出百分比
    int16_t autoModeParam;          // 1006: 自动模式参数
    float rollAngle;                // 1007-1008: 横倾角
    float pitchAngle;               // 1009-1010: 纵倾角
    float rudderAngle;              // 1011-1012: 舵角
    float speed;                    // 1013-1014: 航速
    float timestamp;                // 1015-1016: 时间戳
    double longitude;               // 1017-1020: 经度
    double latitude;                // 1021-1024: 纬度
    float leftEngineSpeed;          // 1025-1026: 左机转速
    float rightEngineSpeed;         // 1027-1028: 右机转速
    int16_t leftEngineGear;         // 1029: 左机挡位
    int16_t rightEngineGear;        // 1030: 右机挡位

    // 状态数据（2001-2023）
    float currentSpeed;             // 2001-2002: 当前航速
    float leftExtendThreshold;      // 2003-2004: 左侧截流板伸出阈值百分比
    float rightExtendThreshold;     // 2005-2006: 右侧截流板伸出阈值百分比
    float leftCurrentExtend;        // 2007-2008: 左侧截流板当前伸出百分比
    float rightCurrentExtend;       // 2009-2010: 右侧截流板当前伸出百分比
    int16_t motorCount;             // 2011: 电机数量
    int16_t motor1Status;           // 2012: 电机1状态码
    int16_t motor2Status;           // 2013: 电机2状态码
    int16_t motor3Status;           // 2014: 电机3状态码
    int16_t motor4Status;           // 2015: 电机4状态码
    int16_t imuStatus;              // 2016: 惯导数据状态码
    int16_t slaveStatus;            // 2017: 下位机状态码
    float currentPitch;             // 2020-2021: 船舶当前纵倾角
    float currentRoll;              // 2022-2023: 船舶当前横倾角

    ModbusData() {
        // 初始化默认值
        controlMode = 1; // 默认手动模式
        manualLeftExtend = 0.0f;
        manualRightExtend = 0.0f;
        autoModeParam = 1; // 默认自适应模式
        rollAngle = 0.0f;
        pitchAngle = 0.0f;
        rudderAngle = 0.0f;
        speed = 0.0f;
        timestamp = 0.0f;
        longitude = 0.0;
        latitude = 0.0;
        leftEngineSpeed = 0.0f;
        rightEngineSpeed = 0.0f;
        leftEngineGear = 0; // 默认空档
        rightEngineGear = 0; // 默认空档

        currentSpeed = 0.0f;
        leftExtendThreshold = 100.0f;
        rightExtendThreshold = 100.0f;
        leftCurrentExtend = 0.0f;
        rightCurrentExtend = 0.0f;
        motorCount = 2; // 默认2个电机
        motor1Status = 101; // 默认正常
        motor2Status = 101; // 默认正常
        motor3Status = 101; // 默认正常
        motor4Status = 101; // 默认正常
        imuStatus = 201; // 默认正常
        slaveStatus = 301; // 默认正常
        currentPitch = 0.0f;
        currentRoll = 0.0f;
    }
};