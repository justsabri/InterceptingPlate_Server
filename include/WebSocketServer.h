#ifndef WEBSOCKET_SERVER_H  // 头文件保护，防止多重包含
#define WEBSOCKET_SERVER_H

#include <websocketpp/config/asio_no_tls.hpp>  // 使用ASIO的非TLS配置
#include <websocketpp/server.hpp>              // WebSocket++服务器库
#include <nlohmann/json.hpp>                   // JSON解析/序列化库
#include <log.h>                               // 日志模块
#include <functional>                          // 函数对象支持
#include <cstdint>                             // 固定大小整数类型
#include <exception>                           // 标准异常
#include <glog/logging.h>
#include <chrono>
#include <set>
#include "event_bus.h"

namespace wspp = websocketpp;  // 命名空间别名简化
using json = nlohmann::json;    // 别名简化json类型
using server = wspp::server<wspp::config::asio>;  // 定义服务器类型

/**
 * @class WebSocketServer
 * @brief WebSocket服务器类，用于管理客户端连接、处理消息和发送数据。
 * 
 * 功能包括：
 *  - 在指定端口上监听并接受客户端连接
 *  - 处理客户端的WebSocket连接打开、关闭、失败事件
 *  - 接收客户端发送的JSON格式消息并解析
 *  - 根据控制模式（手动或自动）处理客户端命令
 *  - 发送模拟数据或响应消息给客户端
 */
class WebSocketServer {
public:
    /**
     * @brief 构造函数，初始化服务器端口和服务器实例
     * @param port 服务器要监听的端口号
     */
    explicit WebSocketServer(EventBus& bus);

    /**
     * @brief 默认析构函数
     */
    ~WebSocketServer() = default;

    /**
     * @brief 启动WebSocket服务器
     * 
     * 在指定端口上开始监听，并启动服务器的I/O循环。
     */
    void run(uint16_t port);

private:
    /**
     * @brief 当有新客户端连接时调用
     * @param hdl 表示客户端连接的句柄
     */
    void on_open(wspp::connection_hdl hdl);

    /**
     * @brief 当接收到客户端消息时调用
     * @param hdl 表示客户端连接的句柄
     * @param msg 接收到的消息智能指针
     */
    void on_message(wspp::connection_hdl hdl, server::message_ptr msg);

    /**
     * @brief 当客户端连接正常关闭时调用
     * @param hdl 表示客户端连接的句柄
     */
    void on_close(wspp::connection_hdl hdl);

    /**
     * @brief 当客户端连接失败时调用
     * @param hdl 表示客户端连接的句柄
     */
    void on_fail(wspp::connection_hdl hdl);
    
    /**
     * @brief 主动向客户端发送消息
     * @param msg 表示需要向所有客户端发送的数据
     */
    void sendToClient(const std::string& msg);

    /**
     * @brief 向客户端发送模拟数据（用于测试或初始化）
     * @param hdl 表示客户端连接的句柄
     */
    void send_mock_data(wspp::connection_hdl hdl);

    /**
     * @brief 处理从客户端接收到的JSON消息
     * @param hdl 表示客户端连接的句柄
     * @param input 解析后的JSON对象
     */
    void process_json_message(wspp::connection_hdl hdl, const json &input);

    /**
     * @brief 发送JSON数据给指定客户端
     * @param hdl 表示客户端连接的句柄
     * @param j 要发送的JSON对象
     */
    void send_json(wspp::connection_hdl hdl, const json &j);

    server server_;     // WebSocket++服务器实例
    EventBus& event_bus;
    std::set<wspp::connection_hdl, std::owner_less<wspp::connection_hdl>> connections;
};

#endif // WEBSOCKET_SERVER_H
