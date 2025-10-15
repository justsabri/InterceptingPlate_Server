cmake_minimum_required(VERSION 3.10)
project(WebSocketServer)

option(ENABLE_VIRTUAL_DEVICES "Enable virtual devices" OFF)
option(ENABLE_MOTOR_MODIFY "Enable manual motor modify" OFF)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#==============================================================================
# “开启调试信息”那里的三行代码必须要有，否则无法对生成的目标文件进行调试。
# 开启调试信息
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wreturn-type -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -Wreturn")
# 原文链接：https://blog.csdn.net/qq_38808215/article/details/89028562
#==============================================================================

# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -rdynamic")
# endif()

# 1. 显式查找线程库
find_package(Threads REQUIRED)

# 查找必需库
find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(glog REQUIRED)
# 包含json和websocketpp
find_package(nlohmann_json 3.0 REQUIRED)
find_package(WebSocketpp REQUIRED)

# 设置源文件
set(SOURCE_FILES
    src/main.cpp
    src/WebSocketServer.cpp
    src/controller.cpp
    src/alg_processor.cpp
    src/pid_control.cpp
    src/data_container.cpp
    src/data_center.cpp
    src/imu_rs232.cpp
    src/monitor_system.cpp
    src/motor_controller.cpp
    src/motor.cpp
    src/pc.cpp
)

# 可执行文件目标
add_executable(websocket_server ${SOURCE_FILES})

# 2. 现代包含目录声明（PRIVATE作用域）
target_include_directories(websocket_server PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 3. 修正库链接
target_link_libraries(websocket_server PRIVATE
    # Boost组件（需要指定具体库）
    Boost::system  
    # 线程库
    Threads::Threads
    # JSON库
    nlohmann_json::nlohmann_json
    # 4. 添加Boost头文件库（可选但推荐）
    Boost::headers
    
    glog::glog
)

if (ENABLE_MOTOR_MODIFY)
    # 电机手动修改参数
    add_executable(motor_menu
        test/test.cpp
        src/motor.cpp
    )
    target_include_directories(motor_menu PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )
    target_link_libraries(motor_menu PRIVATE
        # Boost组件（需要指定具体库）
        Boost::system  
        # 线程库
        Threads::Threads
        # JSON库
        nlohmann_json::nlohmann_json
        # 4. 添加Boost头文件库（可选但推荐）
        Boost::headers

        glog::glog
    )

endif()

if (ENABLE_VIRTUAL_DEVICES)
    # 虚拟设备
    add_executable(virtual_device
        test/virtual_main.cpp
    )
    target_include_directories(virtual_device PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )
    target_link_libraries(virtual_device PRIVATE
        # Boost组件（需要指定具体库）
        Boost::system  
        # 线程库
        Threads::Threads
        # JSON库
        nlohmann_json::nlohmann_json
        # 4. 添加Boost头文件库（可选但推荐）
        Boost::headers

        glog::glog
    )

    # 串口修改为虚拟串口
    target_compile_definitions(websocket_server PUBLIC VIRTUAL_TEST)

endif()
