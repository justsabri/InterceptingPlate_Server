// monitor_system.cpp
#include "monitor_system.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "data_center.h"
#include "motor.h"
#include "controller.h"
#include <thread>
#include <pthread.h>

// 截流板控制实现与测试验证 20250821
// 《截流板控制实现与验证测试策略-修改.docx》
#define PID_CONTROL_TEST 0

// ==================== 全局数据队列实例化 ====================
ThreadSafeQueue<std::map<int, MotorData>> motor_data_queue;
ThreadSafeQueue<ImuData> imu_data_queue;
ThreadSafeQueue<LinuxPcData> pc_data_queue;

// ==================== 线程安全队列实现 ====================
template <typename T>
void ThreadSafeQueue<T>::Push(const T value)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(value);
    }
    cv_.notify_one();
}

template <typename T>
bool ThreadSafeQueue<T>::Pop(T &value, bool block)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (block)
    {
        cv_.wait(lock, [this]
                 { return !queue_.empty() || !running_; });
    }
    if (queue_.empty() || !running_)
        return false;
    
    value = std::move(queue_.front());
    queue_.pop();
    return true;
}

template <typename T>
void ThreadSafeQueue<T>::Stop()
{
    running_ = false;
    cv_.notify_all();
}

// 显式实例化模板类
template class ThreadSafeQueue<MotorData>;
template class ThreadSafeQueue<ImuData>;
template class ThreadSafeQueue<LinuxPcData>;

// ==================== 电机监控线程实现 ====================
MotorMonitorThread::MotorMonitorThread()
{
    // 订阅电机数据
    DataCenter::instance().subscribe<std::map<int, MotorData>>(
        Topic::MotorStatus,
        [this](std::map<int, MotorData> raw_data)
        {
            motor_data_queue.Push(raw_data);
        },
        this);

    namespace fs = std::filesystem;
    fs::path config_path = fs::current_path() / "config/config.json";
    std::ifstream config_file(config_path);
    nlohmann::json config = nlohmann::json::parse(config_file);

    // 安全初始化
    motor_position_offset_.motor_num = 0;
    motor_position_offset_.motor.clear();
    motor_position_offset_.motor_position_offset.clear(); // 清空偏移位置存储

    for (const auto &[motor_id, motor_info] : config["motors"].items())
    {
        // 1. 增加电机计数器
        motor_position_offset_.motor_num++;

        // 2. 存储电机ID（原始字符串）
        motor_position_offset_.motor.push_back(std::stoi(motor_id, nullptr, 16));

        // 3. 获取并存储偏移位置
        // 直接从JSON中获取浮点数值
        float offset = motor_info["motor_offset_positon"].get<double>();
        motor_position_offset_.motor_position_offset.push_back(offset);

        // 4.初始化电机状态
        motor_state_[std::stoi(motor_id, nullptr, 16)].alarm_code = 101;
        // 获取最大转动角度
        motor_position_offset_.max_deg = config["ext2deg"]["x_max"].get<float>() - config["ext2deg"]["x_min"].get<float>();
    }
}

MotorMonitorThread::~MotorMonitorThread()
{
    Stop();
}

void MotorMonitorThread::Start()
{
    AINFO << "41";
    if (!running_)
    {
        AINFO << "42";
        running_ = true;
        AINFO << "43";
        thread_ = std::thread(&MotorMonitorThread::MonitoringLoop, this);
        AINFO << "44";
    }
    AINFO << "45";
}

void MotorMonitorThread::Stop()
{
    if (running_)
    {
        running_ = false;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }
}

std::map<int, MotorStateData> MotorMonitorThread::GetMotorStatus()
{
    // AINFO << "MOTOR DISCONNECT before ";
    std::lock_guard<std::mutex> lock(status_mutex_);
    // AINFO << "MOTOR DISCONNECT " << motor_state_[3].alarm_code << " " << motor_state_[4].alarm_code;
    return motor_state_;
}

void MotorMonitorThread::MonitoringLoop()
{
    pthread_setname_np(pthread_self(), "motor_mt"); // 设置线程名
    // 为每个电机状态添加计数器
    std::map<int, std::map<int, int>> error_counters; // <电机索引, <错误类型, 计数>>
    auto start_time = std::chrono::steady_clock::now();
    while (running_)
    {
        std::map<int, MotorData> motors;

        // 阻塞等待新数据
        if (!motor_data_queue.Pop(motors) || motors.size() != motor_position_offset_.motor_num)
        {
            continue;
        }

        for (int i = 0; i < motors.size(); i++)
        { // 遍历所有电机
            int motor_index = motor_position_offset_.motor[i];
            AINFO << "motor index " << motor_index;
            MotorData data = motors[motor_index];
            motor_state_[motor_index].alarm_code = 101;
            // 电机断连检测
            if (data.disconnect)
            {
                motor_state_[motor_index].alarm_code = 110;
                continue;
            }
            // === 异常检测处理 ===
            // 温度异常检测
            if (data.temperature > 80.0)
            {
                if (++error_counters[motor_index][102] >= 1)
                { // 连续5次检测到异常
                    AWARN << "电机" << motor_index << "温度异常: "
                          << data.temperature << "℃" << std::endl;
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    motor_state_[motor_index].alarm_code = 102;
                }
                else
                {
                    error_counters[motor_index][102] = 0; // 恢复正常时重置计数
                }
            }

            // 状态异常检测
            if (data.status.has_error())
            {
                if (++error_counters[motor_index][103] >= 1)
                {
                    AWARN << "电机" << motor_index << "状态异常" << std::endl;
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    motor_state_[motor_index].alarm_code = 103;
                }
            }
            else
            {
                error_counters[motor_index][103] = 0;
            }

            // 电压异常检测
            if (data.voltage < 45.0 || data.voltage > 50.0)
            {
                if (++error_counters[motor_index][104] >= 3)
                {
                    AWARN << "电机" << motor_index << "电压异常: "
                          << data.voltage << "V" << std::endl;
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    motor_state_[motor_index].alarm_code = 104;
                }
            }
            else
            {
                error_counters[motor_index][104] = 0;
            }

            // 电流超限检测
            if (data.current > 12500.0)
            {
                if (++error_counters[motor_index][105] >= 1)
                {
                    AWARN << "电机" << motor_index << "电流过大: "
                          << data.current << "mA" << std::endl;
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    motor_state_[motor_index].alarm_code = 105;
                }
            }
            else
            {
                error_counters[motor_index][105] = 0;
            }
            AERROR << "电机" << motor_index << "编码器电池电压" << data.encoder_battery_voltage;
            // 电池电压异常检测
            // 获取的电压*0.01为实际电压
            if (data.encoder_battery_voltage >= 290 && data.encoder_battery_voltage <= 320)
            {
                if (++error_counters[motor_index][106] >= 1)
                {
                    AWARN << "电机" << motor_index << "电池电压异常: "
                          << data.encoder_battery_voltage << "V" << std::endl;
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    motor_state_[motor_index].alarm_code = 106;
                }
            }
            else if (data.encoder_battery_voltage < 290)
            {
                if (++error_counters[motor_index][107] >= 1)
                {
                    AWARN << "电机" << motor_index << "电池电压异常: "
                          << data.encoder_battery_voltage << "V" << std::endl;
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    motor_state_[motor_index].alarm_code = 107;
                }
            }
            else
            {
                error_counters[motor_index][106] = 0;
                error_counters[motor_index][107] = 0;
            }
            double current_altitude_offset = (data.position_offset / 6619136.0) * 360.0;
            double motor_offset_positon = (motor_position_offset_.motor_position_offset[i] / 360.0) * 6619136.0;
            //     // 位置偏移异常检测
            //     if (abs(data.position_offset-motor_offset_positon)>= 12000) {
            //         if (++error_counters[motor_index][108] >= 5) {
            //         AWARN << "电机" << motor_index<< "偏移角度======="<<current_altitude_offset<<"=====当前位置角度===="<<data.position<<"====位置偏移异常: "
            //             << data.position_offset << "mm" << std::endl;
            //         std::lock_guard<std::mutex> lock(status_mutex_);
            //         motor_state_[motor_index].status = false;
            //         motor_state_[motor_index].alarm_code = 108;
            //     }
            // }else{
            //      error_counters[motor_index][108] = 0;
            // }
            motor_state_[motor_index].plate = data.position;

            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::steady_clock::now() - start_time)
                               .count();

            if (elapsed < 10)
            {
                // 开机10s内位置监控
                AINFO << "time count " << elapsed << " " << error_counters[motor_index][108] << " " << data.position << "count " << error_counters[motor_index][108];
                if (data.position > 2 && data.position < motor_position_offset_.max_deg + 0.5)
                {
                    static int64_t error_elapsed = 0;
                    if (++error_counters[motor_index][108] >= 1)
                    {
                        AWARN << "电机" << motor_index << "初始位置未回零: "
                              << data.position << "度" << "偏移位置：" << data.position_offset << std::endl;
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        // 使用118来让controller控制电机回0
                        motor_state_[motor_index].alarm_code = 118;
                        error_elapsed = elapsed;
                    }
                    if (elapsed > error_elapsed + 3)
                    {
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        motor_state_[motor_index].alarm_code = 108;
                    }
                }
                else
                {
                    error_counters[motor_index][108] = 0;
                }
            }
            else
            {
                // 开机10s后位置监控
                if (error_counters[motor_index][108] > 0)
                {
                    motor_state_[motor_index].alarm_code = 108;
                }
                else if (data.position < -2 || data.position > motor_position_offset_.max_deg + 2)
                {
                    if (++error_counters[motor_index][109] >= 1)
                    {
                        AWARN << "电机" << motor_index << "位置超限: "
                              << data.position << "度" << "偏移位置：" << data.position_offset << std::endl;
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        if (motor_state_[motor_index].alarm_code != 108)
                        {
                            motor_state_[motor_index].alarm_code = 109;
                        }
                    }
                }
                else
                {
                    error_counters[motor_index][109] = 0;
                }
            }
        }
    }
}

// ==================== 惯导监控线程实现 ====================
ImuMonitorThread::ImuMonitorThread()
{
    // 订阅惯导数据
    DataCenter::instance().subscribe<ImuData>(
        Topic::ImuStatus,
        [this](ImuData raw_data)
        {
             AINFO << "订阅到的惯导数据时间戳: " << raw_data.timestamp;
            imu_data_queue.Push(raw_data);
        },
        this);
    imu_state_.alarm_code = 201;
}

ImuMonitorThread::~ImuMonitorThread()
{
    Stop();
}

void ImuMonitorThread::Start()
{
    if (!running_)
    {
        running_ = true;
        thread_ = std::thread(&ImuMonitorThread::MonitoringLoop, this);
    }
}

void ImuMonitorThread::Stop()
{
    if (running_)
    {
        running_ = false;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }
}

ImuStateData ImuMonitorThread::GetImuStatus()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return imu_state_;
}

void ImuMonitorThread::MonitoringLoop()
{
    pthread_setname_np(pthread_self(), "imu_mt"); // 设置线程名
    //=================================================================================
    if (PID_CONTROL_TEST) // 测试控制算法
    {
        // 测试控制算法
        while (running_)
        {
            ImuData data;

            // 阻塞等待新数据
            if (!imu_data_queue.Pop(data))
                break;
            imu_state_.alarm_code = 201; // 这个代表 惯导正常

            // // 惯导断连检测
            // if (data.disconnect)
            // {
            //     imu_state_.alarm_code = 209;
            //     continue;
            // }
            // === 异常检测处理 ===
            // if (data.temperature > 85.0)
            // {
            //     AWARN << "惯导温度异常: " << data.temperature << "℃" << std::endl;
            //     std::lock_guard<std::mutex> lock(status_mutex_);
            //     imu_state_.alarm_code = 202; // 惯导温度异常
            // }

            // // 计算绝对速度 (1节 = 0.514444 m/s)
            // float abs_velocity = sqrt(pow(data.north_velocity, 2) +
            //                           pow(data.east_velocity, 2));

            //=================================================================================
            double speed_kn = 0.0;            // 模拟的航速  节
            double simu_roll = 0.0;           // 模拟 横向  °
            double simu_pitch = 0.0;          // 模拟 纵向 °
            double simu_current_rudder = 0.0; // 模拟 船舶当前舵角 °

            int time_set = 200; // 惯导数据采集频率xtime_set=速度变换时间 600对应60s  这个 默认为10Hz
            test_time_count++;
            int current_cycle = test_time_count / time_set;
            //=================================================================================
            int test_mode = 1; // 1:航速最优  2:协调转弯  3:纵倾/摇最优  4:横倾/摇最优
            // 20250816 20250821
            switch (test_mode)
            {
            case 1: // 1:航速最优
            {
                //---------------------1、航速最优测试，只变航速-------------------------------------
                /*
                在不同初始航速下，记录截流板伸缩达到的最佳航速对应的伸缩量。以航速为输入，两侧截流板最佳伸缩量为输出。
                从5-45kn，每间隔5kn，作为航速输入，观察截流板的伸缩量。最终的伸缩量，从小到大再变小，其中最大值不超过安全阈值。
                */
                speed_kn = 10.0; // 模拟的航速  节
                break;
            }
            case 2: // 2:协调转弯
            {
                //---------------------2、协调转弯-------------------------------------
                /*
                1、大角度转向  10°以上
                分别给出10°、-15°、22°舵角指令，从5kn-35kn，每隔10kn航速，验证截流板伸缩量。
                其中，转向侧为当前航速最大伸缩量，另外一侧为0。
                2、3.2.2小角度转向（10°以下，不含10°）
                分别给出3°、-5°、8°舵角指令，从5kn-35kn，每隔10kn航速，验证截流板伸缩量。
                其中，转向侧伸缩量发生变化，另外一侧为0。模拟器中，假设截流板单侧每伸出5mm，等效于1°舵角。
                */
                speed_kn = 25;               // 模拟的航速  节
                simu_current_rudder = -15.0; // 模拟 船舶当前舵角 °
                break;
            }
            case 3: // 3:纵倾/摇最优
            {
                //---------------------3、纵倾/摇最优-------------------------------------
                /*
                1、计算当前纵倾/摇角与设定值的差值，并将其转换为PID控制的输入量。
                2、输入不同的航速，模拟器产生不同的纵倾/摇角（从负值逐渐逼近0°），将不同的纵倾/摇角作为算法的输入。
                */
                speed_kn = 15; // 模拟的航速  节
                break;
            }
            case 4: // 4:横倾/摇最优
            {
                //---------------------4、横倾/摇最优-------------------------------------
                /*
                1、计算当前横倾/摇角与设定值的差值，并将其转换为PID控制的输入量。
                2、输入不同的航速，模拟器产生不同的纵倾/摇角（从负值逐渐逼近0°），将不同的纵倾/摇角作为算法的输入。
                */
                speed_kn = 25; // 模拟的航速  节
                break;
            }
            }

            //=================================================================================
            // 模拟角度
            imu_state_.roll = simu_roll;
            imu_state_.pitch = simu_pitch;
            imu_state_.speed = speed_kn;

            //---------------------1、船舶当前舵角-------------------------------------
            // 20250822 田鸿宇 新算法用到  船舶  舵角参数
            // 船舶当前舵角
            imu_state_.current_rudder = simu_current_rudder;
            //---------------------1、船舶当前舵角-------------------------------------
            //=================================================================================
            // 经纬度范围检测
            // if ((data.longitude < 73.0 || data.longitude > 136.0) ||
            //     (data.latitude < 3.0 || data.latitude > 54.0))
            // {
            //     AWARN << "惯导经纬度异常: 经度=" << data.longitude
            //           << "°, 纬度=" << data.latitude << "°" << std::endl;
            //     std::lock_guard<std::mutex> lock(status_mutex_);
            //     imu_state_.alarm_code = 206;
            // }
            // imu_state_.latitude = data.latitude;
            // imu_state_.longitude = data.longitude;

            // int gps_week = data.gps_week;
            // double gps_tow = data.gps_millisecond * 0.001;

            // // 转换为UTC时间戳
            // time_t utcTimestamp = gpsToUtc(gps_week, gps_tow);

            // // 转换为北京时间（UTC+8）
            // time_t beijingTimestamp = utcTimestamp + 8 * 3600;
            // struct tm *beijingTime = gmtime(&beijingTimestamp);

            // // 格式化和输出北京时间（仅到秒）
            // std::stringstream bjSS;
            // bjSS << std::put_time(beijingTime, "%Y-%m-%d %H:%M:%S");
            // // AERROR<<"===========time" << bjSS.str();
            // imu_state_.gps_time = bjSS.str();

            // imu_state_.yaw = data.yaw;
        }
    }
    //=================================================================================
    else
    {   
        AINFO << "进入惯导监控线程正常模式";
        // 为惯导每个状态添加计数器
        std::map<int, int> error_counters; // <错误类型, 计数>
        // 正常 运行
        while (running_)
        {
            ImuData data;
            AINFO << "准备从队列Pop数据...";
            // 阻塞等待新数据
            if (!imu_data_queue.Pop(data))
                break;
            AINFO << "从队列Pop出的数据 - 时间戳: " << data.timestamp << ", 航速: " << data.speed;
            imu_state_.alarm_code = 201;

            // 断连检测
            // AINFO << "imu monitor: " << data.disconnect;
            // if (data.disconnect)
            // {
            //     imu_state_.alarm_code = 209;
            //     continue;
            // }
            // === 异常检测处理 ===
            // if (data.temperature > 68)
            // {
            //     AWARN << "惯导温度异常: " << data.temperature << "℃" << std::endl;
            //     std::lock_guard<std::mutex> lock(status_mutex_);
            //     imu_state_.alarm_code = 202;
            // }

            // 计算绝对速度 (1节 = 0.514444 m/s)
            // float abs_velocity = sqrt(pow(data.north_velocity, 2) +
            //                           pow(data.east_velocity, 2));
            //=================================================================================

            //=================================================================================
            imu_state_.speed = data.speed; // 航速  单位：节
            // AERROR <<"=======惯导监控线程获取航速；"<<imu_state_.speed;
            // 速度超限检测
            // if (data.north_velocity < -40 || data.north_velocity > 40.0 ||
            //         data.east_velocity < -40 || data.east_velocity > 40.0)
            // {
            //     AWARN << "航速异常: 北向速度 " << data.north_velocity << " 东向速度： " << data.east_velocity << "m/s" << std::endl;
            //     std::lock_guard<std::mutex> lock(status_mutex_);
            //     imu_state_.alarm_code = 203;
            // }
            //=================================================================================
            // 新速度检测
            if (data.speed < 0 || data.speed > 80.0)
            {
                AWARN << "航速异常: 航速 " << data.speed << " 节" << std::endl;
                std::lock_guard<std::mutex> lock(status_mutex_);
                imu_state_.alarm_code = 204;
            }
            //=================================================================================
            //=================================================================================
            // 姿态角范围检测
            if (data.pitch < -30 || data.pitch > 30.0)
            {
                AWARN << "惯导姿态角异常:纵摇=" << data.pitch << "°" << std::endl;
                std::lock_guard<std::mutex> lock(status_mutex_);
                imu_state_.alarm_code = 202;
            }
            if (data.roll < -40 || data.roll > 40.0)
            {
                AWARN << "惯导姿态角异常:横摇=" << data.roll << "°" << std::endl;
                std::lock_guard<std::mutex> lock(status_mutex_);
                imu_state_.alarm_code = 203;
            }
            //--------------------------------------------------------
            // 真实角度
            imu_state_.roll = data.roll;
            imu_state_.pitch = data.pitch;
            //=================================================================================
            // 舵角范围检测
            if (data.rudder < -30 || data.rudder > 30.0)
            {
                AWARN << "惯导舵角异常:舵角=" << data.rudder << "°" << std::endl;
                std::lock_guard<std::mutex> lock(status_mutex_);
                imu_state_.alarm_code = 205;
            }
            imu_state_.current_rudder = data.rudder;
            //================================================================================
            // 时间戳检测
            // 时间戳验证（通常不早于2020年，不晚于未来1小时）
            std::time_t current_time = std::time(nullptr);
            if (data.timestamp < 1577836800 || data.timestamp > current_time + 3600)
            {
                AERROR << "时间戳异常，可能无效: " << data.timestamp;
                imu_state_.alarm_code = 206;
            }

            // 转换为可读格式
            AINFO << "监控系统数据: " << data.timestamp<<"";
            std::time_t time_val = data.timestamp;
            char time_buf[64];
            std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&time_val));

            imu_state_.gps_time = std::string(time_buf);
            AINFO << "惯导时间: " << imu_state_.gps_time;
            //================================================================================
            // 主机转速检测
            if (data.rpm < 0 || data.rpm > 5000.0)
            {
                AWARN << "主机转速异常:主机转速=" << data.rpm << "rpm" << std::endl;
                std::lock_guard<std::mutex> lock(status_mutex_);
                imu_state_.alarm_code = 207;
            }
            imu_state_.rpm = data.rpm;
            //================================================================================
            // 艏向角范围检测
            // if (data.yaw < 0 || data.yaw > 360.0) {
            //     AWARN << "惯导艏向角异常: " << data.yaw << "°" << std::endl;
            //     std::lock_guard<std::mutex> lock(status_mutex_);
            //     imu_state_.alarm_code = 205;
            // }

            // // 经纬度范围检测
            // if ((data.longitude < 73.0 || data.longitude > 135.0) ||
            //     (data.latitude < 3.0 || data.latitude > 54.0))
            // {
            //     AWARN << "惯导经纬度异常: 经度=" << data.longitude
            //           << "°, 纬度=" << data.latitude << "°" << std::endl;
            //     std::lock_guard<std::mutex> lock(status_mutex_);
            //     imu_state_.alarm_code = 206;
            // }
            // imu_state_.latitude = data.latitude;
            // imu_state_.longitude = data.longitude;

            // 定位状态检测
            // AINFO << "data.GNSS_staus " << data.GNSS_staus;
            // if (data.GNSS_staus == 0) {
            //     if (++error_counters[207] >= 3) {
            //         AWARN << "定位状态异常" << std::endl;
            //         std::lock_guard<std::mutex> lock(status_mutex_);
            //         imu_state_.alarm_code = 207;
            //     }
            // } else {
            //     error_counters[207] = 0;
            // }

            // // AINFO << "data.posture_status " << data.posture_status;
            // // 姿态状态检测
            // if (data.posture_status == 0) {
            //     if (++error_counters[208] >= 3) {
            //         AWARN << "姿态状态异常" << std::endl;
            //         std::lock_guard<std::mutex> lock(status_mutex_);
            //         imu_state_.alarm_code = 208;
            //     }
            // } else {
            //     error_counters[208] = 0;
            // }

            // int gps_week = data.gps_week;
            // AERROR<<"==========data.gps_week:"<<data.gps_week;
            // AERROR<<"==========gps_week:"<<gps_week;
            // double gps_tow = data.gps_millisecond * 0.001;

            // 转换为UTC时间戳
            // time_t utcTimestamp = gpsToUtc(gps_week, gps_tow);

            // 转换为北京时间（UTC+8）
            // time_t beijingTimestamp = utcTimestamp + 8 * 3600;
            // struct tm *beijingTime = gmtime(&beijingTimestamp);

            // 格式化和输出北京时间（仅到秒）
            // std::stringstream bjSS;
            // bjSS << std::put_time(beijingTime, "%Y-%m-%d %H:%M:%S");
            // AERROR<<"===========time" << bjSS.str();
            // imu_state_.gps_time = bjSS.str();

            // imu_state_.yaw = data.yaw;
        }
    }
    //=================================================================================
}

time_t ImuMonitorThread::gpsToUtc(int gpsWeek, double gpsSeconds)
{
    // 计算总秒数（从GPS起始点开始）
    double totalGpsSeconds = gpsWeek * 7 * 86400 + gpsSeconds;
    // 转换为Unix时间戳并减去闰秒
    return GPS_EPOCH + static_cast<time_t>(totalGpsSeconds) - LEAP_SECONDS;
}

// ==================== Linux微电脑监控线程实现 ====================
LinuxPcMonitorThread::LinuxPcMonitorThread()
{
    // 订阅PC数据
    DataCenter::instance().subscribe<LinuxPcData>(
        Topic::PCStatus,
        [this](LinuxPcData raw_data)
        {
            pc_data_queue.Push(raw_data);
        },
        this);
    pc_state_.alarm_code = 301;
}

LinuxPcMonitorThread::~LinuxPcMonitorThread()
{
    Stop();
}

void LinuxPcMonitorThread::Start()
{
    if (!running_)
    {
        running_ = true;
        thread_ = std::thread(&LinuxPcMonitorThread::MonitoringLoop, this);
    }
}

void LinuxPcMonitorThread::Stop()
{
    if (running_)
    {
        running_ = false;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }
}

PCStateData LinuxPcMonitorThread::GetPCStatus()
{

    std::lock_guard<std::mutex> lock(status_mutex_);
    return pc_state_;
}

void LinuxPcMonitorThread::MonitoringLoop()
{
    pthread_setname_np(pthread_self(), "pc_mt"); // 设置线程名
    while (running_)
    {
        LinuxPcData data;

        // 阻塞等待新数据
        if (!pc_data_queue.Pop(data))
            continue;
        pc_state_.alarm_code = 301;

        // === 主监控项（每100秒更新）===
        // 温度异常检测
        if (data.temperature > 100)
        {
            AWARN << "PC温度异常: " << data.temperature << "℃" << std::endl;
            pc_state_.alarm_code = 302;
        }

        // === CPU监控（每10秒更新）===
        if (data.cpu_usage > 70)
        {
            AWARN << "CPU使用率过高: " << data.cpu_usage << "%" << std::endl;
            pc_state_.alarm_code = 303;
        }

        // 电压异常检测
        // if (data.voltage < 11.5 || data.voltage > 12.5) {
        //     AWARN << "PC电压异常: " << data.voltage << "V" << std::endl;
        //     pc_state_.alarm_code = 304;
        // }

        // 存储空间检测
        if (data.storage_usage > 70.0)
        {
            AWARN << "存储空间不足: " << data.storage_usage << "%" << std::endl;
            pc_state_.alarm_code = 304;
        }
    }
}

// ==================== 监控系统管理类实现 ====================
MonitoringSystem::MonitoringSystem()
    : motor_monitor_(std::make_unique<MotorMonitorThread>()),
      imu_monitor_(std::make_unique<ImuMonitorThread>()),
      pc_monitor_(std::make_unique<LinuxPcMonitorThread>()),
      motor_safety_running_(false)
{
}

MonitoringSystem::~MonitoringSystem()
{
    StopAll();
}

void MonitoringSystem::init(int freq_out, StateCallback cb)
{
    cb_freq_ = freq_out;
    callback_ = cb;
}

void MonitoringSystem::StartAll()
{
    AINFO << "Starting All MonitoringThread..." << std::endl;
    if (!running_)
    {
        running_ = true;
        callback_thread_ = std::thread(&MonitoringSystem::CallbackLoop, this);
    }
    AINFO << "31";
    motor_monitor_->Start();
    AINFO << "32";
    imu_monitor_->Start();
    AINFO << "33";
    pc_monitor_->Start();
    AINFO << "34";

    // 启动航速与电机位置监控线程
    if (!motor_safety_running_)
    {
        motor_safety_running_ = true;
        speed_motor_monitor_thread_ = std::thread(&MonitoringSystem::MotorSafetyMonitorLoop, this);
    }
}

void MonitoringSystem::StopAll()
{
    AINFO << "Stoping All MonitoringThread..." << std::endl;
    motor_monitor_->Stop();
    imu_monitor_->Stop();
    pc_monitor_->Stop();

    // 停止航速与电机位置监控线程
    if (motor_safety_running_)
    {
        motor_safety_running_ = false;
        if (speed_motor_monitor_thread_.joinable())
        {
            speed_motor_monitor_thread_.join();
        }
    }

    if (running_)
    {
        running_ = false;
        if (callback_thread_.joinable())
        {
            callback_thread_.join();
        }
    }
}

void MonitoringSystem::StopAllQueues()
{
    motor_data_queue.Stop();
    imu_data_queue.Stop();
    pc_data_queue.Stop();
}

DataPack MonitoringSystem::GetAllStatus()
{
    DataPack dataPack;
    dataPack.motor_state = motor_monitor_->GetMotorStatus();
    dataPack.imu_state = imu_monitor_->GetImuStatus();
    dataPack.pc_state = pc_monitor_->GetPCStatus();
    return dataPack;
}

// 添加线程执行函数
void MonitoringSystem::CallbackLoop()
{
    using namespace std::chrono;
    auto interval = microseconds(1000000 / cb_freq_);
    auto next_run = steady_clock::now();

    while (running_)
    {
        callback_(GetAllStatus());

        next_run += interval;
        std::this_thread::sleep_until(next_run);
    }
    AINFO << "callback loop stop";
}

void MonitoringSystem::MotorSafetyMonitorLoop()
{
    using namespace std::chrono;
    constexpr auto check_interval = milliseconds(50); // 每50ms检查一次
    // 读取配置
    namespace fs = std::filesystem;
    fs::path config_path = fs::current_path() / "../config/config.json";
    std::ifstream config_file(config_path);
    if (!config_file)
    {
        return;
    }
    nlohmann::json config = nlohmann::json::parse(config_file);

    // 安全初始化
    config_info_.motor_num = 0;
    config_info_.motor.clear();
    config_info_.max_extension = 0;

    for (const auto &[motor_id, motor_info] : config["motors"].items())
    {
        // 1. 增加电机计数器
        config_info_.motor_num++;
        // 2. 存储电机ID（原始字符串）
        config_info_.motor.push_back(std::stoi(motor_id, nullptr, 16));
    }

    config_info_.max_extension = config["max_ext"].get<float>(); // 截流板长度

    while (motor_safety_running_)
    {
        auto start_time = steady_clock::now();

        // 获取当前系统状态
        DataPack current_status = GetAllStatus();
        // 截流板安全阈值（百分比）
        float max_plate_extension = 0;
        //  AERROR << "=============阈值监控线程获取航速"<<current_status.imu_state.speed;
        if (current_status.imu_state.speed < 20)
        {
            max_plate_extension = 1;
        }
        else if (current_status.imu_state.speed >= 20 && current_status.imu_state.speed < 40)
        {
            max_plate_extension = -0.06 * current_status.imu_state.speed + 2.2;
            //-----------会存在小于0------------------
            if (max_plate_extension < 0.0)
            {
                max_plate_extension = 0.0;
            }
        }
        else
        {

            max_plate_extension = 0.0;
        }
        float allowed_extension = config_info_.max_extension * max_plate_extension; // 当前截流板伸出量安全值（截流板长度）

        const double base = 15.0 * SQRT3;
        // 反函数公式：θ = arcsin((y - 15√3)/30) + 60°
        double ratio = (allowed_extension - base) / 30.0;
        // 当前电机安全角度
        double allowed_ratio = std::asin(ratio) * RAD_TO_DEG + 60.0;

        for (int i = 0; i < config_info_.motor_num; i++)
        {
            // 判断当前电机转动角度是否大于安全角度，若是发送指令修改电机角度为安全角度
            if (current_status.motor_state[i].plate > allowed_ratio)
            {
                MotorParser::getInstance().setPositionModeAndTarget(allowed_ratio, config_info_.motor[i]);
            }
        }
        // 计算下次检查时间
        auto elapsed = steady_clock::now() - start_time;
        if (elapsed < check_interval)
        {
            std::this_thread::sleep_for(check_interval - elapsed);
        }
    }
}
// ==================== 主函数 ====================
// int main() {
//     // 创建监控系统管理实例
//     MonitoringSystem monitoring_system;

//     // 启动所有监控线程
//     monitoring_system.StartAll();

//     // 等待退出指令
//     std::cout << "监控系统已启动 (输入q退出)..." << std::endl;
//     while (std::cin.get() != 'q') {}

//     // 安全停止所有组件
//     monitoring_system.StopAllQueues();
//     monitoring_system.StopAll();

//     std::cout << "系统安全关闭" << std::endl;
//     return 0;
// }