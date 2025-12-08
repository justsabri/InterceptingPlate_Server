#ifndef UDP_DATA_SERVER_H
#define UDP_DATA_SERVER_H

#include <cstdint>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include "data_struct.h"
#include "log.h"

class UdpDataServer {
private:
    int server_fd;
    int port;
    std::atomic<bool> is_running;
    struct sockaddr_in server_addr;
    
    // 数据存储和线程安全
    ImuData current_data;
    std::mutex data_mutex;
    
    // 接收线程
    std::thread receiver_thread;
    
    // 私有方法
    void receiverLoop();
    float bigEndianToFloat(const uint8_t* data);
    uint32_t bigEndianToUint32(const uint8_t* data);
    ImuData parseData(const uint8_t* buffer, size_t length);
    bool validateData(const ImuData& data);

public:
    /**
     * 构造函数
     * @param port_num 监听端口，默认9090
     */
    UdpDataServer(int port_num = 9090);
    
    /**
     * 析构函数
     */
    ~UdpDataServer();
    
    // 禁止拷贝构造和赋值
    UdpDataServer(const UdpDataServer&) = delete;
    UdpDataServer& operator=(const UdpDataServer&) = delete;
    
    /**
     * 启动UDP数据服务器（完成所有初始化并开始监听）
     * @return true: 启动成功, false: 启动失败
     */
    bool start();
    
    /**
     * 停止UDP数据服务器
     */
    void stop();
    
    /**
     * 获取当前数据（线程安全）
     * @return 最新的外部信息系统数据
     */
    ImuData getData();
    
    /**
     * 获取服务器状态
     * @return true: 运行中, false: 已停止
     */
    bool isRunning() const { return is_running; }
    
    /**
     * 获取服务器监听端口
     * @return 端口号
     */
    int getPort() const { return port; }
    
    /**
     * 获取服务器状态信息
     * @return 状态描述字符串
     */
    std::string getStatus() const;
};

#endif // UDP_DATA_SERVER_H