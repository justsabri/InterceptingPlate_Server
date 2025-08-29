#include "data_center.h"
#include "log.h"
#include "data_struct.h"
#include <thread>
#include <chrono>
// 单例实例获取方法
DataCenter& DataCenter::instance() {
    static DataCenter dc;  // 首次调用时创建
    return dc;
}

// 构造函数实现
DataCenter::DataCenter() : stop_flag_(false) {
    // 创建工作线程，专门处理发布任务
    worker_thread_ = std::thread(&DataCenter::worker, this);
    
    // 创建DataContainer实例
    data_container_ = std::make_unique<DataContainer>();
    
    // 设置数据回调
    data_container_->motorData([this](const std::map<int, MotorData>& data){
        this->publish(Topic::MotorStatus, data);
    });
    data_container_->imuData([this](const ImuData& data){
        this->publish(Topic::ImuStatus, data);
    });
    data_container_->pcData([this](const LinuxPcData& data) {
        this->publish(Topic::PCStatus, data);
    });
}

// 析构函数实现
DataCenter::~DataCenter() {
    {
        // 先停止数据收集线程
        if (data_container_) {
            data_container_->stop_threads();
        }
        
        // 通知工作线程停止
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stop_flag_ = true;
    }
    queue_cond_.notify_one();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

// 工作线程实现
void DataCenter::worker() {
    while (true) {
        // 定义一个任务对：<主题, 数据>
        std::pair<Topic, std::any> task;
        
        {
            // 使用unique_lock以便与条件变量配合
            std::unique_lock<std::mutex> lock(queue_mutex_);
            
            // 等待条件：任务队列非空或收到停止信号
            queue_cond_.wait(lock, [this] {
                return !publish_queue_.empty() || stop_flag_;
            });
            
            // 检查是否需要终止线程
            if (stop_flag_ && publish_queue_.empty()) {
                return;  // 退出线程函数
            }
            
            // 从队列中取出任务
            if (!publish_queue_.empty()) {
                task = std::move(publish_queue_.front());  // 使用移动语义
                publish_queue_.pop();  // 移除已取出的任务
            }
        }
        
        // 如果有有效任务（task.second包含数据）
        if (task.second.has_value()) {
            // 执行实际的发布操作
            processTask(task.first, std::move(task.second));
        }
    }
}

// 任务处理实现
void DataCenter::processTask(Topic topic, std::any data) {
    // 获取该主题的所有订阅者回调（快速复制）
    std::vector<std::pair<void*, Callback>> callbacks;
    {
        // 锁定订阅者列表（注意：此锁不与队列锁同时持有）
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = topic_subscribers_.find(topic);
        if (it == topic_subscribers_.end()) return;  // 无订阅者直接返回
        callbacks = it->second;  // 复制回调列表
    }
    
    // 遍历所有回调并执行
    for (auto& cb : callbacks) {
        try {
            cb.second(data);  // 执行订阅者回调
        }
        catch (const std::exception& e) {
            // 处理标准异常
            std::cerr << "Callback exception: " << e.what() << std::endl;
        }
        catch (...) {
            // 处理未知异常
            std::cerr << "Unknown callback exception" << std::endl;
        }
    }
}

// 发布数据实现
void DataCenter::publish(Topic topic, std::any data) {
    {
        // 锁定任务队列
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // 将任务添加到队列
        publish_queue_.emplace(topic, std::move(data));
    }
    // 通知工作线程有新任务
    queue_cond_.notify_one();
}

// 注册收集器实现
void DataCenter::registerCollector(Topic topic, CollectorCallback collector) {
    std::lock_guard<std::mutex> lock(collector_mutex_);
    collectors_[topic] = std::move(collector);
}

// 解除注册收集器实现
void DataCenter::unregisterCollector(Topic topic) {
    std::lock_guard<std::mutex> lock(collector_mutex_);
    collectors_.erase(topic);
}

// 初始化数据收集线程实现
void DataCenter::initDataContainer(int motor_freq, int imu_freq, int pc_freq) {
    // AINFO <<"container device start";
    // InitDeviceStatus if_Device_init;
    // InitDeviceStatus retry_Device_init;
     data_container_->initDevice();
     data_container_->init_threads(motor_freq, imu_freq, pc_freq);
    // // AINFO <<"container thread start";
    // if (if_Device_init.imu == 0 && if_Device_init.motor == 0)
    // {
    //     data_container_->init_threads(motor_freq, imu_freq, pc_freq);
    // }
    // else{
    //     //未成功初始化设备，延迟5s重新初始化
    //     std::this_thread::sleep_for(std::chrono::seconds(5));
    //     retry_Device_init = data_container_->initDevice();
    //     if (retry_Device_init.imu == 0 && retry_Device_init.motor == 0)
    //     {
    //          data_container_->init_threads(motor_freq, imu_freq, pc_freq);
    //     }
    //     else{
    //         if(retry_Device_init.imu != 0){

    //         }
    //         else if(retry_Device_init.motor != 0){

    //         } 
    //     }
    // }
   
}