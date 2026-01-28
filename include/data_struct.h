#pragma once
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <cstdint>

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
    float left_rudder;  // 左桨舵角 (-30°~30°)
    float right_rudder; // 右桨舵角 (-30°~30°)
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
  float left_rudder;  // 左桨舵角 (-30°~30°)
  float right_rudder; // 右桨舵角 (-30°~30°)
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
