#ifndef DATACENTER_H
#define DATACENTER_H

#include <iostream>
#include <mutex>
#include <any>
#include <unordered_map>
#include <queue>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>
#include "data_container.h"

/**
 * @brief 主题类型枚举：定义不同种类的传感器数据主题
 */
enum class Topic {
    MotorStatus,      ///< 电机状态数据
    ImuStatus,        ///< 惯性测量单元(IMU)状态数据
    PCStatus,         ///< 下位机(PC)状态数据
};

/**
 * @class DataCenter
 * @brief 数据中心类（单例模式）
 *
 * 负责管理数据的发布/订阅，协调各模块间的数据通信
 */
class DataCenter {
private:
    /**
     * @brief 私有构造函数（确保单例模式）
     *
     * 初始化数据结构并创建工作线程
     */
    DataCenter();

    /// @brief 析构函数（释放资源并停止工作线程）
    ~DataCenter();
    
    // 禁用拷贝构造函数和赋值操作符（单例要求）
    DataCenter(const DataCenter&) = delete;
    DataCenter& operator=(const DataCenter&) = delete;

    /// @brief 工作线程函数：持续从任务队列中取出并处理发布任务
    void worker();
    
    /// @brief 处理发布任务：调用订阅者的回调函数
    void processTask(Topic topic, std::any data);

    /// @brief 回调函数类型定义：接受std::any类型参数
    using Callback = std::function<void(const std::any&)>;
    
    /// @brief 数据收集回调函数类型
    using CollectorCallback = std::function<std::any()>;

    // 订阅者管理
    std::unordered_map<Topic, std::vector<std::pair<void*, Callback>>> topic_subscribers_; ///< 订阅者映射表
    mutable std::mutex mutex_; ///< 订阅者列表的互斥锁
    
    // 任务队列管理
    std::queue<std::pair<Topic, std::any>> publish_queue_; ///< 发布任务队列
    std::mutex queue_mutex_; ///< 任务队列的互斥锁
    std::condition_variable queue_cond_; ///< 队列条件变量
    
    // 线程控制
    std::atomic<bool> stop_flag_; ///< 停止标志
    std::thread worker_thread_; ///< 工作线程对象
    
    // 数据收集管理
    std::unordered_map<Topic, CollectorCallback> collectors_; ///< 数据收集器映射表
    std::mutex collector_mutex_; ///< 收集器映射的互斥锁
    
    // 数据容器接口
    std::unique_ptr<DataContainer> data_container_; ///< DataContainer实例指针

public:
    /**
     * @brief 获取单例实例
     * @return DataCenter& 单例对象的引用
     *
     * 使用Meyer's单例实现（C++11线程安全）
     */
    static DataCenter& instance();
    
    /**
     * @brief 订阅指定主题的数据
     * @tparam T 期望的数据类型
     * @param topic 要订阅的主题
     * @param handler 数据处理回调函数
     * @param ptr 对象指针
     *
     * 添加类型安全的回调包装器到订阅列表。
     * 当发布的数据类型与T匹配时，调用用户handler。
     */
    template<typename T>
    void subscribe(Topic topic, std::function<void(T)> handler, void* ptr) {
        // 锁定订阅者列表
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 创建类型安全的回调包装器
        auto wrapper = [handler](const std::any& data) {
            try {
                // 检查类型是否匹配
                if (data.type() == typeid(T)) {
                    // 转换并执行用户回调
                    handler(std::any_cast<T>(data));
                }
            }
            catch (const std::bad_any_cast& e) {
                // 类型转换失败处理
                std::cerr << "Type mismatch: " << e.what() << std::endl;
            }
        };
        
        // 添加到主题的订阅列表
        topic_subscribers_[topic].push_back(std::pair<void*, Callback>(ptr, wrapper));
    }

   /**
     * @brief 取消订阅指定主题的数据
     * @tparam T 期望的数据类型
     * @param topic 要取消订阅的主题
     * @param ptr 对象指针
     *
     * 添加类型安全的回调包装器到订阅列表。
     * 当发布的数据类型与T匹配时，调用用户handler。
     */
    template<typename T>
    void unsubscribe(Topic topic, void* ptr) {
        // 锁定订阅者列表
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 删除相应指针、主题的订阅
        auto itor = std::find_if(topic_subscribers_[topic].begin(), topic_subscribers_[topic].end(),
                          [ptr](const std::pair<void*, std::function<void(const std::any&)>>& item) {
                              return item.first == ptr;  // 比对 void* 指针
                          });
        if (itor != topic_subscribers_[topic].end()) {
            topic_subscribers_[topic].erase(itor);  // 删除迭代器指向的元素
        }
    } 

    /**
     * @brief 发布数据到指定主题
     * @param topic 目标主题
     * @param data 要发布的数据（自动包装为std::any）
     *
     * 线程安全的非阻塞操作。将任务添加到队列后立即返回。
     */
    void publish(Topic topic, std::any data);

    /**
     * @brief 注册数据收集器回调函数
     * @param topic 目标主题
     * @param collector 数据收集回调函数
     * 
     * 允许外部模块注册数据收集器，用于按需获取最新数据。
     * 线程安全：使用互斥锁保护collectors_映射表。
     */
    void registerCollector(Topic topic, CollectorCallback collector);

    /**
     * @brief 解除注册数据收集器
     * @param topic 目标主题
     * 
     * 从指定主题移除已注册的数据收集器（如果存在）。
     * 线程安全：使用互斥锁保护collectors_映射表。
     */
    void unregisterCollector(Topic topic);

    /**
     * @brief 初始化数据收集线程
     * @param motor_freq 电机数据采集频率（Hz）
     * @param imu_freq 惯性测量单元数据采集频率（Hz）
     * @param pc_freq 下位机状态数据采集频率（Hz）
     */
    void initDataContainer(int motor_freq, int imu_freq, int pc_freq);
};

#endif // DATACENTER_H