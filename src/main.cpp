#include "WebSocketServer.h"
#include "ModbusTcpServer.h"
#include <log.h>
#include <cstdlib>
#include "controller.h"
/**
 * @brief 程序主入口
 * 
 * 初始化日志系统，创建并启动WebSocket服务器。
 * 
 * @return 程序退出状态码：0表示正常退出，非0表示异常退出。
 */
int main() {
#ifdef WEBSOCKET_COMMUNICATION
    // 初始化日志系统，设置应用名称
    namespace fs = std::filesystem;
    fs::path log_path = fs::current_path() / "log";
    InitLog("WebSocketServer", log_path.string().c_str(), 10);
    AINFO<<"start system";
    try {
        // 创建事件总线
        EventBus event_bus;
        // 创建WebSocket服务器实例
	    WebSocketServer ws_server(event_bus);
        AERROR << "====================1";
	    // 创建controller实例
        Controller controller(event_bus);
	    AERROR << "====================2";
        // 启动 Controller，触发消息发送
        controller.start();
        AERROR << "====================3";
        // 启动 WebSocket 服务
        ws_server.run(8080);
        AERROR << "====================4";
        
    } catch (const std::exception & e) {
        // 捕获并记录未被处理的异常
        AERROR << "Fatal error: " << e.what();
        // 停止日志系统
        StopLog();
        return EXIT_FAILURE;
    }

    // 正常退出前停止日志系统
    StopLog();

#elif MODBUSTCP_COMMUNICATION
// 初始化日志系统，设置应用名称
    InitLog("ModbusTcpServer");
    AINFO<<"start system";
    try {
        // 创建事件总线
        EventBus event_bus;
        // 创建WebSocket服务器实例
	    ModbusServer ms_server(event_bus);
        AERROR << "====================1";
	    // 创建controller实例
        Controller controller(event_bus);
	    AERROR << "====================2";
        // 启动 Controller，触发消息发送
        controller.start();
        AERROR << "====================3";
        // 启动 modbus 服务
        ms_server.start(502);
        AERROR << "====================4";
        
    } catch (const std::exception & e) {
        // 捕获并记录未被处理的异常
        AERROR << "Fatal error: " << e.what();
        // 停止日志系统
        StopLog();
        return EXIT_FAILURE;
    }

    // 正常退出前停止日志系统
    StopLog();
#endif
    return EXIT_SUCCESS;
}
