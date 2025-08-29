#include "data_container.h"
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <filesystem>
#include <fstream>
#include "log.h"
#include <nlohmann/json.hpp>
//返回值：0 初始化成功  非0：初始化异常
InitDeviceStatus DataContainer::initDevice(void){
    // AINFO << "init Device";
    init_device_status_.motor = MotorParser::getInstance().init("can0");
    AWARN << "初始化电机："<<init_device_status_.motor;
    //从配置文件中获取电机数量与电机id
    namespace fs = std::filesystem;
    fs::path config_path = fs::current_path() / "config/config.json";
    std::ifstream config_file(config_path);
    nlohmann::json config = nlohmann::json::parse(config_file);
    motor_config_.motor_num = 0;
    motor_config_.motor.clear();
    for (const auto& [motor_id, motor_info] : config["motors"].items()) {
    motor_config_.motor_num++;  // 统计电机总数
    // 存储电机ID
    motor_config_.motor.push_back(std::stoi(motor_id, nullptr, 16));
    }
      //清除电机错误状态并将电机恢复至0位
    for (size_t i = 0; i < motor_config_.motor_num; i++)
    {
        MotorParser::getInstance().send(motor_config_.motor[i],0x0B,{});
        MotorParser::getInstance().setPositionModeAndTarget(0,motor_config_.motor[i]);
        // MotorParser::getInstance().setMaxForwardPosition(113,motor_config_.motor[i]);
        // MotorParser::getInstance().flush(motor_config_.motor[i]);
    }
    //  /dev/ttyS8  参照imu_rs232.h中定义修改
    init_device_status_.imu = imu.init("/dev/ttyS8",115200);
    AWARN << "初始化惯导"<<init_device_status_.imu;
    return init_device_status_;
}

void DataContainer::motorData(MotorDataCallback cb) {
    callback_motor = std::move(cb);
}
void DataContainer::imuData(ImuDataCallback cb) {
    callback_imu = std::move(cb);
}
void DataContainer::pcData(PcDataCallback cb) {
    callback_pc = std::move(cb);
}

int DataContainer::refreshMotorData(void) {
    for (size_t i = 0; i < motor_config_.motor_num; i++)
    {
        motor_data_[i] = MotorParser::getInstance().getMotorData(motor_config_.motor[i]);
    }
    // motor_data_[0] = MotorParser::getInstance().getMotorData(0x001);
    // motor_data_[1] = MotorParser::getInstance().getMotorData(0x002);
    // motor_data_[2] = MotorParser::getInstance().getMotorData(0x003);
    // motor_data_[3] = MotorParser::getInstance().getMotorData(0x004);
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (callback_motor) {
        callback_motor(motor_data_);
    }
    return 0;
}

int DataContainer::refreshImuData(void){   
    if(imu.readData() == 0){
        imu_data_ = imu.getdata();  //单次获取并解析采集数据
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (callback_imu) {
            callback_imu(imu_data_);
        }
        return 0;
    }
    return 1;
}

void DataContainer::refreshPcData(void){
    linux_pc_data_.cpu_usage = pc.getCPUUsage();
    linux_pc_data_.storage_usage = pc.getMemoryUsage();
    linux_pc_data_.temperature = pc.getCPUTemperature();
    linux_pc_data_.voltage = pc.getCPUVotage();
    // AERROR<<"==============采集电压："<<linux_pc_data_.voltage;
     std::lock_guard<std::mutex> lock(pc_mutex_);
    if (callback_pc) {
       callback_pc(linux_pc_data_);
    }
}

// 添加线程执行函数
void DataContainer::motor_thread() {
    pthread_setname_np(pthread_self(), "motor_t"); // 设置线程名
    using namespace std::chrono;
    auto interval = microseconds(1000000 / motor_freq_hz_);
    auto next_run = steady_clock::now();

    while (running_) {
        refreshMotorData();
    
        next_run += interval;
        std::this_thread::sleep_until(next_run);
    }
}

void DataContainer::imu_thread() {
    using namespace std::chrono;
    auto interval = microseconds(1000000 / imu_freq_hz_);
    auto next_run = steady_clock::now();

    while (running_) {
        refreshImuData();
        
        next_run += interval;
        std::this_thread::sleep_until(next_run);
    }
}

void DataContainer::pc_thread() {
    using namespace std::chrono;
    auto interval = microseconds(1000000 / pc_freq_hz_);
    auto next_run = steady_clock::now();

    while (running_) {
        refreshPcData();
        
        next_run += interval;
        std::this_thread::sleep_until(next_run);
    }
}

//初始化函数
void DataContainer::init_threads(int motor_freq, int imu_freq, int pc_freq) {
    // 设置获取频率(Hz)
    if (motor_freq > 0) motor_freq_hz_ = motor_freq;
    if (imu_freq > 0) imu_freq_hz_ = imu_freq;
    if (pc_freq > 0) pc_freq_hz_ = pc_freq;
    
    // 启动采集线程
    running_ = true;
    AINFO<<"start collect motor";
    motor_thread_ = std::thread(&DataContainer::motor_thread, this);
    AINFO<<"start collect imu";
    imu_thread_ = std::thread(&DataContainer::imu_thread, this);
    AINFO<<"start collect pc";
    pc_thread_ = std::thread(&DataContainer::pc_thread, this);
}

void DataContainer::stop_threads() {
    running_ = false;
    if (motor_thread_.joinable()) {
        motor_thread_.join();
    }
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }

    if (pc_thread_.joinable()) {
        pc_thread_.join();
    }
}
