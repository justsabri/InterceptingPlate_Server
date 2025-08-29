// monitor_system.h
#ifndef MONITOR_SYSTEM_H
#define MONITOR_SYSTEM_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <queue>
#include <condition_variable>
#include <functional>
#include <memory>
#include <utility>
#include "log.h"
#include <nlohmann/json.hpp>
#include "data_struct.h"
#include <ctime>
#include <iomanip>
#include <sstream>

// 定义GPS起始时间：1980-01-06 00:00:00 UTC
const time_t GPS_EPOCH = 315964800; // Unix时间戳（秒）

// 获取当前闰秒偏移量（截至2023年为18秒）
const int LEAP_SECONDS = 18;

// ==================== 线程安全队列模板 ====================
/**
 * @brief 线程安全队列模板类
 *
 * 提供线程安全的队列操作，支持阻塞和非阻塞方式
 *
 * @tparam T 队列元素类型
 */
template <typename T>
class ThreadSafeQueue {
public:
	ThreadSafeQueue() = default;

	// 禁止拷贝和赋值
	ThreadSafeQueue(const ThreadSafeQueue&) = delete;
	ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

	/**
	 * @brief 添加数据到队列
	 *
	 * @param value 要添加的数据
	 */
	void Push(const T value);

	/**
	 * @brief 从队列获取数据
	 *
	 * @param value 获取的数据存储位置
	 * @param block 是否阻塞等待（默认为true）
	 * @return true 成功获取数据
	 * @return false 队列已停止或为空
	 */
	bool Pop(T& value, bool block = true);

	/**
	 * @brief 停止队列操作
	 *
	 * 唤醒所有等待线程并停止队列
	 */
	void Stop();

private:
	std::queue<T> queue_;              ///< 内部队列
	std::mutex mutex_;                 ///< 互斥锁
	std::condition_variable cv_;       ///< 条件变量
	std::atomic<bool> running_{ true };  ///< 队列运行状态
};

// ==================== 全局数据队列 ====================
extern ThreadSafeQueue<std::map<int, MotorData>> motor_data_queue;
extern ThreadSafeQueue<ImuData> imu_data_queue;
extern ThreadSafeQueue<LinuxPcData> pc_data_queue;

// ==================== 监控线程基类 ====================
/**
 * @brief 监控线程基类
 *
 * 提供线程启动、停止的基本功能
 */
class MonitorThreadBase {
public:
	virtual ~MonitorThreadBase() = default;

	/**
	 * @brief 启动监控线程
	 */
	virtual void Start() = 0;

	/**
	 * @brief 停止监控线程
	 */
	virtual void Stop() = 0;

protected:
	std::atomic<bool> running_{ false }; ///< 线程运行状态
	std::thread thread_;               ///< 线程对象
	
};

// ==================== 电机监控线程类 ====================
/**
 * @brief 电机监控线程类
 *
 * 负责监控电机系统的各项参数
 */
class MotorMonitorThread : public MonitorThreadBase {
public:
	MotorMonitorThread();
	~MotorMonitorThread();

	void Start() override;
	void Stop() override;

	std::map<int, MotorStateData> GetMotorStatus();
private:
	/**
	 * @brief 主监控循环
	 *
	 * 处理电机数据并执行异常检测
	 */
	void MonitoringLoop();
	std::mutex status_mutex_;
	std::map<int, MotorStateData> motor_state_; // 存储4个电机的状态

    typedef struct {
        int motor_num;
        std::vector<int> motor;
		std::vector<double> motor_position_offset;
    } motor_position_offset;

	motor_position_offset motor_position_offset_;
	
};

// ==================== 惯导监控线程类 ====================
/**
 * @brief 惯导监控线程类
 *
 * 负责监控惯导系统的各项参数
 */
class ImuMonitorThread : public MonitorThreadBase {
public:
	ImuMonitorThread();
	~ImuMonitorThread();

	void Start() override;
	void Stop() override;

	ImuStateData GetImuStatus();
	time_t gpsToUtc(int gpsWeek, double gpsSeconds);

private:
	/**
	 * @brief 主监控循环
	 *
	 * 处理惯导数据并执行异常检测
	 */
	void MonitoringLoop();
	ImuStateData imu_state_;
	std::mutex status_mutex_;

// ------------------------------------------------------------------------
//航速测试用
    int test_time_count = 0;
// ------------------------------------------------------------------------
};

// ==================== Linux微电脑监控线程类 ====================
/**
 * @brief Linux微电脑监控线程类
 *
 * 负责监控Linux微电脑的各项参数
 */
class LinuxPcMonitorThread : public MonitorThreadBase {
public:
	LinuxPcMonitorThread();
	~LinuxPcMonitorThread();

	void Start() override;
	void Stop() override;

	PCStateData GetPCStatus();

private:

	PCStateData pc_state_;
	std::mutex status_mutex_;	
	/**
	 * @brief 主监控循环
	 *
	 * 处理PC数据并执行异常检测
	 */
	void MonitoringLoop();
};

// ==================== 监控系统管理类 ====================
/**
 * @brief 监控系统管理类
 *
 * 统一管理所有监控线程的启动和停止
 */
class MonitoringSystem {
public:
	MonitoringSystem();
	~MonitoringSystem();

	using StateCallback = std::function<void(DataPack)>;
	/**
	 * @brief 初始化
	 * @param freq_out: 监控模块输出数据频率
	 * @param freq_in: 监控模块数据输入频率，及数据中心分发频率
	 */
	void init(int freq_out, StateCallback cb);

	/**
	 * @brief 启动所有监控线程
	 */
	void StartAll();

	/**
	 * @brief 停止所有监控线程
	 */
	void StopAll();

	/**
	 * @brief 停止所有数据队列
	 */
	void StopAllQueues();

	//从三个线程中获取系统状态
	DataPack GetAllStatus();

	//截流板安全阈值监控
	void MotorSafetyMonitorLoop();

private:
	void CallbackLoop();
	std::unique_ptr<MotorMonitorThread> motor_monitor_;  ///< 电机监控线程
	std::unique_ptr<ImuMonitorThread> imu_monitor_;      ///< 惯导监控线程
	std::unique_ptr<LinuxPcMonitorThread> pc_monitor_;   ///< PC监控线程
	StateCallback callback_;
	std::thread callback_thread_;
	int cb_freq_;
	std::atomic<bool> running_{ false };
	bool motor_safety_running_ = false;                   // 截流板安全阈值监控线程运行标志
	std::thread speed_motor_monitor_thread_;             // 截流板安全阈值监控线程

	typedef struct {
        int motor_num;
		std::vector<int> motor;
        float max_extension;
    } Config_Info;

	Config_Info config_info_;
};

#endif // MONITOR_SYSTEM_H
