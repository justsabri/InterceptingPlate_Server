# JLB-10 15m 协议测试与联调说明

## 1. 测试范围

协议单元测试目标为 `protocol_15m_test`，覆盖：

- 消息 0：控制模式、自动模式参数、手动伸缩比例、时间戳，以及非法控制模式。
- 消息 1：固定 46 字节帧长度、payload 长度不含 4 字节消息头、2 电机默认状态和 4 电机状态。
- 消息 2：float、double、int16、时间戳字段及非法 payload 长度。
- TCP 流：半包保留、粘包拆分、消息 0/2 连续帧提取。
- TCP 异常恢复：非法消息类型或 payload 长度会丢弃字节并继续寻找下一帧。

## 2. 构建和运行单元测试

### 2.1 直接编译

在仓库目录执行：

```bash
g++ -std=c++17 -I include \
  test/protocol_15m_test.cpp src/protocol_15m.cpp \
  -o protocol_15m_test

./protocol_15m_test
```

Windows PowerShell 可执行：

```powershell
g++ -std=c++17 -I include test/protocol_15m_test.cpp src/protocol_15m.cpp -o protocol_15m_test.exe
.\protocol_15m_test.exe
Remove-Item .\protocol_15m_test.exe
```

预期输出：

```text
protocol_15m_test passed
```

### 2.2 CMake 构建

Linux 环境需要先安装项目依赖，包括 CMake、Boost、glog、nlohmann-json 和 WebSocketpp。然后执行：

```bash
cmake -S . -B build
cmake --build build --target protocol_15m_test -j
./build/protocol_15m_test
```

`protocol_15m_test` 只依赖协议头文件和 `src/protocol_15m.cpp`，不需要 socket、glog 或业务设备。

## 3. 构建 tcp_server

```bash
cmake -S . -B build
cmake --build build --target tcp_server -j
```

运行前确认当前工作目录是 `InterceptingPlate_Server`，因为程序按相对路径读取：

```text
config/config.json
```

启动示例：

```bash
./build/tcp_server
```

TCP 监听端口以 `src/main.cpp` 的实际配置为准。查看端口：

```bash
rg "TcpServer|start\\(" src/main.cpp src
```

消息 1 的发送周期由 `cb_freq` 驱动。15m TCP 模式要求 20Hz：

```json
{
    "cb_freq": 20
}
```

即使配置误写为其他值，`TCP_COMMUNICATION` 构建路径也会在启动时将其覆盖为 20，并记录告警。

## 4. TCP 客户端模拟器

模拟器位于：

```text
test/15m_tcp_client_sim.py
```

它使用大端序生成消息 0/2，并支持：

- `normal`：分别发送完整消息。
- `half`：将消息 0 拆成两次发送。
- `sticky`：一次发送消息 0 和消息 2。
- `invalid`：先发送非法帧，再发送合法帧。
- `disconnect`：发送半帧后主动断开。

运行示例：

```bash
python3 test/15m_tcp_client_sim.py --host 127.0.0.1 --port 2000 --scenario normal
python3 test/15m_tcp_client_sim.py --host 127.0.0.1 --port 2000 --scenario half
python3 test/15m_tcp_client_sim.py --host 127.0.0.1 --port 2000 --scenario sticky
python3 test/15m_tcp_client_sim.py --host 127.0.0.1 --port 2000 --scenario invalid
python3 test/15m_tcp_client_sim.py --host 127.0.0.1 --port 2000 --scenario disconnect
```

模拟器会读取服务器返回的消息 1 头部和关键字段，并输出：

- 实际接收字节数。
- 消息类型。
- payload 长度。
- 航速。
- 电机数量。
- IMU / 从机状态码。

## 5. 抓包校验

消息头统一为 4 字节大端序：

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 2 | 消息类型 |
| 2 | 2 | payload 长度，不含消息头 |

消息 0：

- 总长度：24 字节。
- payload 长度：20 字节。
- 控制模式：偏移 4，uint16。
- 自动模式参数：偏移 6，uint16。
- 左侧手动比例：偏移 8，float。
- 右侧手动比例：偏移 12，float。
- 时间戳：偏移 16，uint64。

消息 1：

- 总长度：46 字节。
- payload 长度：42 字节。
- 航速：偏移 4，float。
- 左/右阈值：偏移 8 / 12，float。
- 左/右当前伸缩量：偏移 16 / 20，float。
- 电机数量：偏移 24，uint16。
- 电机 1 至 4 状态：偏移 26 / 28 / 30 / 32，uint16。
- IMU / 从机状态：偏移 34 / 36，uint16。
- 当前纵摇 / 横摇：偏移 38 / 42，float。

消息 2：

- 总长度：56 字节。
- payload 长度：52 字节。
- 横摇、纵摇、舵角、航速：偏移 4 / 8 / 12 / 16，float。
- 经度、纬度：偏移 20 / 28，double。
- 左/右主机转速：偏移 36 / 40，float。
- 左/右挡位：偏移 44 / 46，int16。
- 时间戳：偏移 48，double。

Wireshark 原始字节检查时，先确认消息类型和 payload 长度，再按上述偏移解释字段。禁止把前 4 字节消息头计入 payload 长度。

## 6. 20Hz 状态发送校验

在客户端保持连接至少 2 秒，记录收到的消息 1 时间。消息 1 固定长度应为 46 字节，理论间隔约为 50ms：

```text
period = timestamp[i + 1] - timestamp[i]
```

实际系统会受调度、网络缓冲和客户端读取方式影响，建议统计至少 40 个样本，用平均周期判断是否接近 50ms，而不要用单个间隔判定。

同时确认：

- 2 电机配置下消息 1 仍为 46 字节。
- 电机 3 / 4 状态字段为初始值 101。
- 消息 1 不因消息 2 的接收频率改变长度。
- 非法输入不会触发控制命令或覆盖有效状态。

## 7. Git 分支与 PR 流程

本系列按依赖顺序使用 stacked branch：

```text
feature/JLB-5-15m-protocol-codec
feature/JLB-6-15m-control-command
feature/JLB-7-15m-ship-status-chain
feature/JLB-8-15m-status-20hz
feature/JLB-9-15m-tcp-stream-buffer
feature/JLB-10-15m-tests-docs
```

提交前执行：

```bash
git status --short --branch
git diff --check
```

PR 描述应写明：

- 变更覆盖的消息类型和测试场景。
- 本地实际运行的测试命令及输出。
- 是否完成完整 `tcp_server` 构建。
- stacked PR 的 base 分支及后续 retarget 计划。