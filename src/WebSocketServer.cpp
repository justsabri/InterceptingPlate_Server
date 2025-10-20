#include "WebSocketServer.h"

// 构造函数实现
WebSocketServer::WebSocketServer(EventBus& bus) : event_bus(bus) {
    AERROR << &event_bus;
    // 使用ASIO初始化服务器
    server_.init_asio();
    
    // 设置地址重用，以便在服务器重启时可以快速重新绑定端口
    server_.set_reuse_addr(true);
    
    // 关闭所有访问日志和错误日志（生产环境可根据需要调整）
    server_.clear_access_channels(wspp::log::alevel::all);
    server_.clear_error_channels(wspp::log::elevel::all);
    
    // 设置连接打开事件的处理函数
    server_.set_open_handler([this](wspp::connection_hdl hdl) {
        on_open(hdl);
    });
    
    // 设置消息接收事件的处理函数
    server_.set_message_handler([this](wspp::connection_hdl hdl, server::message_ptr msg) {
        on_message(hdl, msg);
    });
    
    // 设置连接关闭事件的处理函数
    server_.set_close_handler([this](wspp::connection_hdl hdl) {
        on_close(hdl);
    });
    
    // 设置连接失败事件的处理函数
    server_.set_fail_handler([this](wspp::connection_hdl hdl) {
        on_fail(hdl);
    });

    // 注册回调
    event_bus.subscribe<json>("to_ws", [this](const json j) {
        std::string msg = j.dump();
        AINFO << "send by websocket";
        sendToClient(msg);
    });
}

// 启动服务器
void WebSocketServer::run(uint16_t port) {
    // 初始化日志：记录服务器正在初始化的端口
    AINFO << "Initializing WebSocket server on port:" << port;

    try {
        // 开始监听指定端口
        server_.listen(port);
        // 开始接受新的客户端连接
        server_.start_accept();
        // 运行服务器I/O循环
        server_.run();
    } catch (const std::exception & e) {
        // 记录启动失败的错误信息
        AERROR << "Failed to start server: " << e.what();
    }
}

// 当有新客户端连接时的处理
void WebSocketServer::on_open(wspp::connection_hdl hdl) {
    // 记录新客户端连接
    AINFO << "New client connected";
    // 向新客户端发送模拟数据
    connections.insert(hdl);
    // send_mock_data(hdl);
}

// 处理接收到的客户端消息
void WebSocketServer::on_message(wspp::connection_hdl hdl, server::message_ptr msg) {
    try {
        // 获取连接指针，以便记录远程端点信息
        auto con = server_.get_con_from_hdl(hdl);
        // 在日志中记录控制请求
        AINFO << "request from " << con->get_remote_endpoint();
        // 打印接收到的原始消息（调试级别）
        ADEBUG << "Received message: " << msg->get_payload();
        // 解析JSON格式的消息
        auto j = json::parse(msg->get_payload());
        // 解析收到的消息并通过 event_bus 向 Controller 发送数据
        // 将从 WebSocket 客户端接收到的消息传递给 Controller 进行处理
        event_bus.publish("from_ws", j);
        if (j["cmd"] == "ping") {
            

            // 响应客户端的 ping 消息
            nlohmann::json reply = {{"cmd", "pong"}};
            sendToClient(reply.dump());
        }
    } catch (const json::parse_error & e) {
        // JSON解析失败的处理
        AWARN << "JSON parse error: " << e.what();
    } catch (const std::exception & e) {
        // 其他消息处理异常的处理
        AERROR << "Message processing error: " << e.what();
    }
}

// 当客户端连接关闭时的处理
void WebSocketServer::on_close(wspp::connection_hdl hdl) {
    // 获取连接指针，以便获取远程端点信息
    auto con = server_.get_con_from_hdl(hdl);
    connections.erase(hdl);

    json j;
    j["command"]["control_mode"] = "STOP";
    event_bus.publish("from_ws", j);
    // 记录客户端断开连接
    AINFO << "Client disconnected: " << con->get_remote_endpoint();
}

// 当客户端连接失败时的处理
void WebSocketServer::on_fail(wspp::connection_hdl hdl) {
    // 获取连接指针，以便获取远程端点信息
    auto con = server_.get_con_from_hdl(hdl);
    // 记录连接失败
    AERROR << "Connection failed from " << con->get_remote_endpoint();
}

// 向客户端发送数据
void WebSocketServer::sendToClient(const std::string& msg) {
    for (auto& hdl : connections) {
        server_.send(hdl, msg, websocketpp::frame::opcode::text);
    }
}

// 向客户端发送模拟数据
void WebSocketServer::send_mock_data(wspp::connection_hdl hdl) {
    AINFO << "Sending mock data to client";
    // 创建模拟数据JSON对象
    json mock_data;

         // 构造导航数据
    mock_data["data"]["navigation"]["enable"] = true;
    mock_data["data"]["navigation"]["latitude"] = 15.4;
    mock_data["data"]["navigation"]["longitude"] = 12.2;
    mock_data["data"]["navigation"]["speed"] = 10.1;
    
    // 设置错误代码
    mock_data["data"]["alarm_code"] = 101;
    
    // 构造截流板状态数据
    mock_data["data"]["interceptor_status"]["enable"] = true;
    mock_data["data"]["interceptor_status"]["extension"]["plate_1"] = 0.0;
    mock_data["data"]["interceptor_status"]["extension"]["plate_2"] = 0.0;
    mock_data["data"]["interceptor_status"]["current_ratio"]["plate_1"] = 0.0;
    mock_data["data"]["interceptor_status"]["current_ratio"]["plate_2"] = 0.0;
    
    // 构造设备状态数据
    mock_data["data"]["devices"]["enable"] = true;
    mock_data["data"]["devices"]["motor_num"] = 4;
    // 模拟4个电机的状态，第一个正常，后三个为false（异常或关闭）
    mock_data["data"]["devices"]["motors"] = { true, false, false, false };
    mock_data["data"]["devices"]["imu"] = true;
    mock_data["data"]["devices"]["controller"] = true;
    
    // 构造姿态数据
    mock_data["data"]["attitude"]["enable"] = true;
    mock_data["data"]["attitude"]["pitch_deg"] = 4.5;
    mock_data["data"]["attitude"]["roll_deg"] = 5.5;
    
    // 添加高精度时间戳（毫秒级）
    auto now = std::chrono::system_clock::now();
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    mock_data["timestamp"] = ts;
    // 发送模拟数据给客户端
    send_json(hdl, mock_data);
  
    
}

// 处理JSON格式的控制消息
void WebSocketServer::process_json_message(wspp::connection_hdl hdl, const json &input) {
    // 获取连接指针，以便记录远程端点信息
    auto con = server_.get_con_from_hdl(hdl);
    try {
        // 从JSON对象中获取command对象
        const json& command = input["command"];
        // 获取控制模式（MANUAL或AUTO）
        std::string controlMode = command["control_mode"].get<std::string>();

        // 根据控制模式进行分支处理
        if (controlMode == "MANUAL") {
            // 获取手动控制参数
            const json& manual_params = command["manual_params"];
            // 获取截流板伸长度参数
            const json& extension = manual_params["extension"];
            
            // 解析两个截流板的设定值
            float plate_1 = extension["plate_1"].get<float>();
            float plate_2 = extension["plate_2"].get<float>();
            
            // 在日志中记录手动控制请求
            AINFO << "MANUAL mode request from " << con->get_remote_endpoint()
                   << ": plate_1=" << plate_1 << ", plate_2=" << plate_2;

            // 构建并发送响应消息
            json response;
            response["response"] = "MANUAL";
            send_json(hdl, response);
        }
        else if (controlMode == "AUTO") {
            // 获取自动模式参数（这里假设是一个整数模式）
            int mode = command["auto_params"].get<int>();
            // 在日志中记录自动控制请求
            AINFO << "AUTO mode request from " << con->get_remote_endpoint()
                   << ": mode=" << mode;
            
            // 构建并发送响应消息
            json response;
            response["response"] = "AUTO";
            send_json(hdl, response);
        }
        else {
            // 记录无效的控制模式
            AWARN << "Invalid control mode from " << con->get_remote_endpoint()
                   << ": " << controlMode;
            throw std::runtime_error("Invalid control mode: " + controlMode);
        }
    } catch (const std::exception & e) {
        // 记录命令处理中的异常
        AERROR << "Command processing error: " << e.what();
    }
}

// 发送JSON数据给客户端
void WebSocketServer::send_json(wspp::connection_hdl hdl, const json &j) {
    try {
        // 将JSON对象序列化为字符串
        std::string payload = j.dump();
        // 获取连接指针，用于日志记录
        auto con = server_.get_con_from_hdl(hdl);
        // 记录发送消息的信息（包括接收端和消息大小）
        AINFO << "Sending message to " << con->get_remote_endpoint() 
               << " (" << payload.size() << " bytes)";
        // 通过WebSocket发送消息（文本帧）
        server_.send(hdl, payload, wspp::frame::opcode::text);
        // 在调试日志中记录发送的消息内容（生产环境可能需关闭）
        ADEBUG << "Sent JSON message: " << payload;
    } catch (const wspp::exception & e) {
        // 发送过程中出现错误的处理
        AERROR << "Error sending message: " << e.what();
    }
}