#include "controller.h"
#include <iostream>
#include <log.h>
#include <filesystem>
#include <fstream>
#include "data_center.h"

Controller::Controller(EventBus& bus) : event_bus_(bus), thread_pool_(1),
                                        auto_mode_(0) {
    AERROR << &event_bus_;
#ifdef WEBSOCKET_COMMUNICATION
    event_bus_.subscribe<nlohmann::json>("from_ws", [this](const json j) {
        handle_message(j);  // 当 WebSocket 服务端发来数据时，进行处理
    });
#elif MODBUSTCP_COMMUNICATION
    event_bus_.subscribe<ModbusDataEvent>("from_modbus", [this](const ModbusDataEvent event) {
        handle_message(event);
    });
#endif
    alg_processor_ = std::make_unique<AlgProcessor>();
    motor_ctrl_ = std::make_unique<MotorController>();

    ctl_params_.auto_mode = 0xFF;
    ctl_params_.auto_mode = 0xFF;
}

void Controller::start() {
    // 读取配置
    namespace fs = std::filesystem;

    fs::path config_path = fs::current_path() / "config/config.json";
    AINFO << "JSON" << config_path << "cur " << fs::current_path();
    
    if (!fs::exists(config_path)) {
        AINFO << "Config file does not exist: " << config_path;
        return;
    }
    // config_path = "/home/zlg/server/0730/InterceptingPlate_new/config/config.json";
    std::ifstream config_file(config_path);
    if (!config_file) {
        AINFO << "JSON fail" << config_path << "cur " << fs::current_path();
        return;
    }
    nlohmann::json config = nlohmann::json::parse(config_file);
    memset(&config_info_, 0, sizeof(Config_Info));
    for (const auto& [motor_id, motor_info] : config["motors"].items()) {
        config_info_.motor_num++;
        config_info_.motor_id.push_back(std::stoi(motor_id, nullptr, 16));
        if (motor_info["location"] == "left") {

		config_info_.left_motor.push_back(std::stoi(motor_id,nullptr,16));

        } else if (motor_info["location"] == "right") {

		config_info_.right_motor.push_back(std::stoi(motor_id,nullptr,16));

        }
    }
    AINFO << "num " << config_info_.motor_num << "num addr " << std::hex << &config_info_.motor_num;
    config_info_.max_ext = config["max_ext"].get<float>();
    config_info_.ext2deg.a = config["ext2deg"]["a"].get<float>();
    config_info_.ext2deg.b = config["ext2deg"]["b"].get<float>();
    config_info_.ext2deg.x_min = config["ext2deg"]["x_min"].get<float>();
    config_info_.ext2deg.x_max = config["ext2deg"]["x_max"].get<float>();
  
    int motor_freq = config["motor_freq"].get<int>();

    int imu_freq = config["imu_freq"].get<int>();

    int pc_freq = config["pc_freq"].get<int>();

    int cb_freq = config["cb_freq"].get<int>();
 
    AINFO << "==========Left motor IDs:";
    for (auto id : config_info_.left_motor) {
    AINFO << id;
    }
    AINFO << "==========Right motor IDs:";
    for (auto id : config_info_.right_motor) {
    AINFO << id;
    }
    // 初始化算法
    std::function<void(AlgResult&)> alg_cb = [this](AlgResult res) {
        thread_pool_.enqueue([this](const AlgResult res) {
            AERROR<<"调用自动算法"<<res.new_left<<res.new_right;
            excuteAlgCmd(res);
        }, res);    
    };

    alg_processor_->set_callback(alg_cb);
   

   
    // 初始化数据中心回调函数
    DataCenter::instance().initDataContainer(motor_freq, imu_freq, pc_freq);
    motor_data_cb = [this](std::map<int, MotorData> data) {
        alg_package_.motor_data = data;
        AWARN<<"传入电机数据=====================";
        tryProcess();
    };
    
    imu_data_cb = [this](ImuData data) {
        alg_package_.imu_data = data;
         AWARN<<"传入惯导数据=====================";
        tryProcess();
    };
  
     // 初始化电机执行
    motor_ctrl_ = std::make_unique<MotorController>();
    // motor_ctrl_->init_motor_reverse_position();

    // 初始化状态监测
    std::function<void(DataPack)> state_cb = [this](DataPack data) {
        {
            std::lock_guard<std::mutex> l(data_mutex_);
            monitor_pack_ = data;
        }
        sendDataToClient(STATE_DATA, (void*)&monitor_pack_);
        tryHandleError();
    };

    monitor_system_ = std::make_unique<MonitoringSystem>();
    monitor_system_->init(cb_freq, state_cb);
 
    monitor_system_->StartAll();
   
     // 启动一个处理线程
    alg_worker_= std::thread(&AlgProcessor::process_data, alg_processor_.get());
}

void Controller::sendDataToClient(Data_Type type, void* data) {
#ifdef WEBSOCKET_COMMUNICATION
    thread_pool_.enqueue([this](Data_Type type, void* data){
        json j;
        convertStructToJson(type, data, j);
        AINFO << "send data to client";
        AINFO << j.dump(4);
        event_bus_.publish("to_ws", j);  // 发布事件，WebSocket 服务端会收到并主动发送到客户端
        AERROR << "END";
    }, type, data);
#elif MODBUSTCP_COMMUNICATION
    // 不需要主动发送
#endif
}

void Controller::handle_message(const json& j) {
    auto func = [this](const json& j) {
        try {
            // 从JSON对象中获取command对象
            const json& command = j["command"];
            // 获取控制模式（MANUAL或AUTO）
            std::string controlMode = command["control_mode"].get<std::string>();

            // 根据控制模式进行分支处理
            if (controlMode == "STOP") {
                auto_mode_ = 0;
                // 清除自动模式的资源
                DataCenter::instance().unsubscribe<ImuData>(Topic::ImuStatus, this);
                DataCenter::instance().unsubscribe<std::map<int, MotorData>>(Topic::MotorStatus, this);
            } else if (controlMode == "MANUAL") {
                // 清除自动模式的资源
                DataCenter::instance().unsubscribe<ImuData>(Topic::ImuStatus, this);
                DataCenter::instance().unsubscribe<std::map<int, MotorData>>(Topic::MotorStatus, this);
                alg_processor_->clear();
                auto_mode_ = 0;

                // 获取手动控制参数
                const json& manual_params = command["manual_params"];
                // 获取截流板伸长度参数
                const json& extension = manual_params["extension"];
                
                // 解析两个截流板的设定值
                float plate_1 = extension["plate_1"].get<float>();
                float plate_2 = extension["plate_2"].get<float>();
               
                // 在日志中记录手动控制请求
 
                AWARN << "==========MANUAL mode " << ": plate_1=" << plate_1 << ", plate_2=" << plate_2;
                double theta_1 = yToTheta(plate_1*config_info_.max_ext);
                double theta_2 = yToTheta(plate_2*config_info_.max_ext);
                AWARN <<"=========角度："<<theta_1<<"========"<<theta_2;
                // 控制 moter_ctrl执行对应命令
                ctrl_motor(theta_1, theta_2);
                json j_MANNUAL;
                j_MANNUAL["response"] = "MANUAL";
                event_bus_.publish("to_ws", j_MANNUAL);
            } else if (controlMode == "AUTO") {
                // 获取自动模式参数（这里假设是一个整数模式）
                int mode = command["auto_params"].get<int>();
                AINFO << "AUTO mode " << auto_mode_ << ": mode=" << mode;
                json j_AUTO;
                j_AUTO["response"] = "AUTO";
                j_AUTO["mode"] = mode;
                event_bus_.publish("to_ws", j_AUTO);
                if (auto_mode_ == 0 && mode != 0) {
                    auto_mode_ = mode;
                    AERROR<<"进入自动模式=================================";
                    // 向数据中心注册算法topic数据和data_cb，等待数据中心回数据,数据中心回数据后立即push给算法，等待算法结果
                    DataCenter::instance().subscribe<ImuData>(Topic::ImuStatus, imu_data_cb, this);
                    DataCenter::instance().subscribe<std::map<int, MotorData>>(Topic::MotorStatus, motor_data_cb, this);
                    AERROR<<"注册回调成功=================================";
                } else if (mode == 0 && auto_mode_ != 0) {
                    // 清除自动模式的资源
                    DataCenter::instance().unsubscribe<ImuData>(Topic::ImuStatus, this);
                    DataCenter::instance().unsubscribe<std::map<int, MotorData>>(Topic::MotorStatus, this);
                    alg_processor_->clear();
                    auto_mode_ = 0;
                } else if (mode != auto_mode_) {
                    auto_mode_ = mode;
                    // 清除自动模式的资源
                    DataCenter::instance().unsubscribe<ImuData>(Topic::ImuStatus, this);
                    DataCenter::instance().unsubscribe<std::map<int, MotorData>>(Topic::MotorStatus, this);
                    // 控制 moter_ctrl执行对应命令
                    ctrl_motor(0, 0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1300));
                    // 向数据中心注册算法topic数据和data_cb，等待数据中心回数据,数据中心回数据后立即push给算法，等待算法结果
                    DataCenter::instance().subscribe<ImuData>(Topic::ImuStatus, imu_data_cb, this);
                    DataCenter::instance().subscribe<std::map<int, MotorData>>(Topic::MotorStatus, motor_data_cb, this);
                }
            
            }
            else if(controlMode == "PoweOff"){
                system("sudo shut down -h now");
            }
            else {
                // 记录无效的控制模式
                AWARN << "Invalid control mode " << controlMode;
                throw std::runtime_error("Invalid control mode: " + controlMode);
            }
        } catch (const std::exception & e) {
            // 记录命令处理中的异常
            AERROR << "Command processing error: " << e.what();
        }
    };

    thread_pool_.enqueue(func, j);
}

static void auto_ctrl(void* ptr) {
    if (!ptr)
        return;
    Controller* ctl = (Controller*)ptr;
    AINFO << "modbus auto ctrl " << ctl->ctl_params_.mode << " " << ctl->ctl_params_.auto_mode << " " << ctl->last_ctl_params_.mode;
    ctl->last_ctl_params_.mode = ctl->ctl_params_.mode;
    if (ctl->ctl_params_.mode == 0 && ctl->last_ctl_params_.mode != 0) {
        // 清除自动模式的资源
        DataCenter::instance().unsubscribe<ImuData>(Topic::ImuStatus, ctl);
        DataCenter::instance().unsubscribe<std::map<int, MotorData>>(Topic::MotorStatus, ctl);
        // 控制 moter_ctrl执行对应命令
        ctl->ctrl_motor(0, 0);
    } else if (ctl->ctl_params_.mode == 1) {
        if (ctl->ctl_params_.auto_mode == 0xFF) {
            return;
        }
        ctl->auto_mode_ = ctl->ctl_params_.auto_mode;
        if (ctl->last_ctl_params_.mode == 1) {
            return;
        }
        DataCenter::instance().unsubscribe<ImuData>(Topic::ImuStatus, ctl);
        DataCenter::instance().unsubscribe<std::map<int, MotorData>>(Topic::MotorStatus, ctl);
        // 控制 moter_ctrl执行对应命令
        ctl->ctrl_motor(0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1300));
        
        // 向数据中心注册算法topic数据和data_cb，等待数据中心回数据,数据中心回数据后立即push给算法，等待算法结果
        DataCenter::instance().subscribe<ImuData>(Topic::ImuStatus, ctl->imu_data_cb, ctl);
        DataCenter::instance().subscribe<std::map<int, MotorData>>(Topic::MotorStatus, ctl->motor_data_cb, ctl);
    }
}

void Controller::handle_message(const ModbusDataEvent &event) {
    static ModbusParamItem modbus_param_table[] = {
        {"mode", &ctl_params_.mode, 0x1001, auto_ctrl},
        {"ctl_ext_left", &ctl_params_.ext_left, 0x1002, [](void* ptr){
            Controller* ctl = (Controller*)ptr;
            if (std::fabs(ctl->ctl_params_.ext_left - ctl->last_ctl_params_.ext_left) < 0.001) {
                return;
            }
            AINFO << "modbus ext " << std::fabs(ctl->ctl_params_.ext_left - ctl->last_ctl_params_.ext_left);
            ctl->last_ctl_params_.ext_left = ctl->ctl_params_.ext_left;
            double degree = ctl->yToTheta(ctl->ctl_params_.ext_left * ctl->config_info_.max_ext);
            AINFO << "modbus " << degree << " " << ctl->ctl_params_.ext_left;
            ctl->ctrl_motor(degree, std::nullopt);
        }},
        {"ctl_ext_right", &ctl_params_.ext_right, 0x1004, [](void* ptr){
            Controller* ctl = (Controller*)ptr;
            if (std::fabs(ctl->ctl_params_.ext_right - ctl->last_ctl_params_.ext_right) < 0.001) {
                return;
            }
            ctl->last_ctl_params_.ext_right = ctl->ctl_params_.ext_right;
            double degree = ctl->yToTheta(ctl->ctl_params_.ext_right * ctl->config_info_.max_ext);
            AINFO << "modbus " << degree << " " << ctl->ctl_params_.ext_right;
            ctl->ctrl_motor(std::nullopt, degree);
        }},
        {"auto_mode", (void*)(uint8_t*)&ctl_params_.auto_mode, 0x1006, auto_ctrl},
        {"rudder", (void*)(uint8_t*)&ctl_params_.rudder, 0x1007, [](void* ptr){
            Controller* ctl = (Controller*)ptr;
            AINFO << "modbus rudder" << ctl->ctl_params_.rudder;
            if (ctl->alg_package_.imu_data.has_value()) {
                ctl->alg_package_.imu_data.value().rudder = ctl->ctl_params_.rudder;
                AINFO << "modbus " << ctl->ctl_params_.rudder;
            }
        }},
        {"speed", (void*)((uint8_t*)&monitor_pack_ + offsetof(DataPack, imu_state) + offsetof(ImuStateData, speed)), 0x2001, nullptr},
        {"ext_left_limit", (void*)(uint8_t*)&monitor_pack_.motor_state[config_info_.left_motor[0]].plate, 0x2003, nullptr},
        {"ext_right_limit", (void*)(uint8_t*)&monitor_pack_.motor_state[config_info_.right_motor[0]].plate, 0x2005, nullptr},
        {"ext_left", (void*)(uint8_t*)&monitor_pack_.motor_state[config_info_.left_motor[0]].plate, 0x2007, nullptr},
        {"ext_right", (void*)(uint8_t*)&monitor_pack_.motor_state[config_info_.right_motor[0]].plate, 0x2009, nullptr},
        {"motor_num", (void*)(uint8_t*)&config_info_.motor_num, 0x2011, nullptr},
        {"motor1_state", (void*)(uint8_t*)&monitor_pack_.motor_state[1].alarm_code, 0x2012, nullptr},
        {"motor2_state", (void*)(uint8_t*)&monitor_pack_.motor_state[2].alarm_code, 0x2013, nullptr},
        {"motor3_state", (void*)(uint8_t*)&monitor_pack_.motor_state[3].alarm_code, 0x2014, nullptr},
        {"motor4_state", (void*)(uint8_t*)&monitor_pack_.motor_state[4].alarm_code, 0x2015, nullptr},
        {"imu_state", (void*)(uint8_t*)&monitor_pack_.imu_state.alarm_code, 0x2016, nullptr},
        {"pc_state", (void*)(uint8_t*)&monitor_pack_.pc_state.alarm_code, 0x2017, nullptr},
        // {"yaw", (void*)(uint8_t*)&monitor_pack_.imu_state.yaw, 0x2018, nullptr},
        {"pitch", (void*)(uint8_t*)&monitor_pack_.imu_state.pitch, 0x2020, nullptr},
        {"roll", (void*)(uint8_t*)&monitor_pack_.imu_state.roll, 0x2022, nullptr},
    };


    AINFO << event.func << " " << std::hex << event.addr << " " << event.count;
    for (int i = 0; i < sizeof(modbus_param_table)/sizeof(ModbusParamItem); i++) {
        ModbusParamItem& item = modbus_param_table[i];
        // AINFO << "addr: " << item.modbus_addr << " " << event.addr;
        if (item.modbus_addr == event.addr) {
            if (event.func == "POST") {
                memcpy(item.pointer, event.frame, event.len);

                if (event.len == 4) {
                    int32_t value =
                        static_cast<int32_t>(event.frame[3]) |
                        (static_cast<int32_t>(event.frame[2]) << 8) |
                        (static_cast<int32_t>(event.frame[1]) << 16) |
                        (static_cast<int32_t>(event.frame[0]) << 24);
                    AINFO << "creply4 " << static_cast<float>(value);
                    AINFO << "creply4 " << std::hex << static_cast<int32_t>(event.frame[0]) << " "
                          << std::hex << static_cast<int32_t>(event.frame[1]) << " "
                          << std::hex << static_cast<int32_t>(event.frame[2]) << " "
                          << std::hex << static_cast<int32_t>(event.frame[3]);
                }

                if (item.handler_ptr) {
                    thread_pool_.enqueue([this, item](Controller* ctl){item.handler_ptr(ctl); }, this);
                }
            } else if (event.func == "GET") {
                AINFO << item.name << " " <<std::hex<< item.pointer;
                if (item.name == "ext_left") {
                    float v = thetaToY(monitor_pack_.motor_state[config_info_.left_motor[0]].plate) / config_info_.max_ext;
                    AINFO << "V " << v;
                    memcpy(event.frame, &v, event.len);
                } else if (item.name == "ext_right") {
                    float v = thetaToY(monitor_pack_.motor_state[config_info_.right_motor[0]].plate) / config_info_.max_ext;
                    memcpy(event.frame, &v, event.len);
                    AINFO << "V " << v;
                } else if (item.name == "ext_left_limit" || item.name == "ext_right_limit") {
                    float v = safe_ext_.getMaxExtensionRatio(monitor_pack_.imu_state.speed);
                    memcpy(event.frame, &v, event.len);
                    AINFO << "V " << v;
                } else {
                    memcpy(event.frame, item.pointer, event.len);
                }

                uint16_t num = event.frame[1] << 8 | event.frame[0];
                AINFO << "creply " << static_cast<int>(num);
                if (event.len == 4) {
                    int32_t value =
                        static_cast<int32_t>(event.frame[3]) |
                        (static_cast<int32_t>(event.frame[2]) << 8) |
                        (static_cast<int32_t>(event.frame[1]) << 16) |
                        (static_cast<int32_t>(event.frame[0]) << 24);
                    AINFO << "creply4 " << static_cast<float>(value) << " " << *(float*)item.pointer;
                    AINFO << std::hex << static_cast<int32_t>(event.frame[0]) << " "
                          << std::hex << static_cast<int32_t>(event.frame[1]) << " "
                          << std::hex << static_cast<int32_t>(event.frame[2]) << " "
                          << std::hex << static_cast<int32_t>(event.frame[3]);
                }
            }
            break;
        }
    }
}

void Controller::convertStructToJson(Data_Type type, void* data, json &j) {
    std::lock_guard<std::mutex> l(data_mutex_);
    if (data == nullptr)
        return;

    switch(type) {
        case STATE_DATA:
            {
     
                AINFO << "MOTOR ";
                DataPack pack = *(DataPack*)(data);
                AINFO << "MOTOR " << config_info_.motor_num << " " << pack.motor_state.size();
                j["data"]["navigation"] = {
                    {"enable", true},
                    {"speed", pack.imu_state.speed}
                };
                alg_package_.in.max_extension = safe_ext_.getMaxExtensionRatio(pack.imu_state.speed) * config_info_.max_ext;
                alg_package_.in.current_speed = pack.imu_state.speed;
                alg_package_.in.pitch_current = pack.imu_state.pitch;
                alg_package_.in.heel_current = pack.imu_state.roll;
                alg_package_.in.left_current =  thetaToY(pack.motor_state[config_info_.left_motor[0]].plate);
                alg_package_.in.right_current = thetaToY(pack.motor_state[config_info_.right_motor[0]].plate) ;
                // ------------------------------------------------------------------------
                //测试用截流板安全阈值
                float test_plate_extension = 0.0;
                AWARN<<"=============测试速度："<<pack.imu_state.speed;
                // if(pack.imu_state.speed< 20){
                //     test_plate_extension = 1.0;
                // }
                // else if(pack.imu_state.speed >= 20.0 && pack.imu_state.speed < 40.0){
                //     test_plate_extension = -0.06*pack.imu_state.speed + 2.2;
                // }
                // ------------------------------------------------------------------------
                if(pack.imu_state.speed < 20){
                    test_plate_extension = 1.0;
                }
                else if(pack.imu_state.speed >= 20.0 && pack.imu_state.speed < 40.0){
                    //  这可能为负的了 20250814 
                    test_plate_extension = -0.06*pack.imu_state.speed + 2.2;
                    //-----------会存在小于0------------------
                    if(test_plate_extension < 0.0)
                    {
                        test_plate_extension = 0.0;
                    }
                }
                else{
                    test_plate_extension = 0.0;
                }
                // test_plate_extension = 1.0;
                AWARN<<"===============截流板伸出量阈值百分比："<<test_plate_extension*100.0;
                // ------------------------------------------------------------------------
                j["data"]["interceptor_status"] = {
                    {"enable", true},
                    {"extension", {
                        // {"plate_1", 0.0},
                        // {"plate_2", 0.0}
                        {"plate_1", test_plate_extension},
                        {"plate_2", test_plate_extension}
                    }},
                    {"current_ratio", {
                        {"plate_1", thetaToY(pack.motor_state[config_info_.left_motor[0]].plate) / config_info_.max_ext},
                        {"plate_2", thetaToY(pack.motor_state[config_info_.right_motor[0]].plate) / config_info_.max_ext}
                    }}
                };
                AWARN<<"=============左侧返回角度："<<pack.motor_state[config_info_.left_motor[0]].plate<<"  返回百分比： "
                    <<100.0*thetaToY(pack.motor_state[config_info_.left_motor[0]].plate) / config_info_.max_ext;


                std::vector<int> m_a_c;
                for (int i = 0; i < config_info_.motor_num; i++)
                {
                    m_a_c.push_back(pack.motor_state[config_info_.motor_id[i]].alarm_code);
                }
                
                j["data"]["devices"] = {
                    {"enable", true},
                    {"motor_num", config_info_.motor_num},
                    {"motor_alarm_code",m_a_c},
                    {"imu_alarm_code",pack.imu_state.alarm_code},
                    {"controller_alarm_code",pack.pc_state.alarm_code}
                };

                j["data"]["attitude"] = {
                    {"enable", true},
                    {"pitch_deg", pack.imu_state.pitch},
                    {"roll_deg", pack.imu_state.roll}
                };

                j["data"]["timestamp"] = pack.imu_state.gps_time;
                
                break;
            }
        default:
            break;
    }
}

//-------------------------------------------------------------------------------
// 纵倾/摇最优  模拟角度  根据伸出量  来变化 20250822

// 最优值为-2°，对应伸缩量为30mm
double simulation_angle(double x) {
    if (x < 0 || x > 50) {
        return 0; // Return NaN if x is out of range
    } else if (x <= 10) {
        return -10 + 0.7 * x;
    } else if (x <= 20) {
        return 2 - 0.5 * x;
    } else if (x <= 30) {
        return 0.6 * x - 20;
    } else {
        return 7 - 0.3 * x;
    }
}


// // 最优值为-2°，对应伸缩量为15mm
// double simulation_angle(double x) {
//     if (x < 0 || x > 50) {
//         return 0;
//     } else if (x <= 15) {
//         // 从(0,-11)到(15,-2)的线性插值
//         return -11 + 0.6 * x; // 斜率 = (-2 - (-11)) / (15 - 0) = 9/15 = 0.6
//     } else if (x <= 20) {
//         // 从(15,-2)到(20,-5)的线性插值
//         return -2 - 0.6 * (x - 15); // 斜率 = (-5 - (-2)) / (20 - 15) = -3/5 = -0.6
//     } else if (x <= 30) {
//         // 从(20,-5)到(30,-3)的线性插值
//         return -5 + 0.2 * (x - 20); // 斜率 = (-3 - (-5)) / (30 - 20) = 2/10 = 0.2
//     } else {
//         // 从(30,-3)到(50,-9)的线性插值
//         return -3 - 0.3 * (x - 30); // 斜率 = (-9 - (-3)) / (50 - 30) = -6/20 = -0.3
//     }
// }




// double simulation_angle(int speed, double x) {
//     // 根据航速和等效截流板伸缩量计算船舶角度
//     // 参数 speed: 航速（单位knots）
//     // 参数 x: 等效截流板伸缩量（单位mm）
//     // 返回: 船舶角度（单位度）
    
//     double a = 0.0, b = 0.0, c = 0.0, d = 0.0, e = 0.0, f = 0.0;

//     if (speed == 5) {
//         a = -2E-07;
//         b = 2E-05;
//         c = -0.001;
//         d = 0.0195;
//         e = -1.46;
//         f = 0;
//     }
//     else if (speed == 10) {
//         a = -1E-07;
//         b = 1E-05;
//         c = -0.0005;
//         d = 0.0139;
//         e = -2.06;
//         f = 0;
//     }
//     else if (speed == 15) {
//         a = -0.0036;
//         b = -0.1279;
//         c = 0.778;
//         d = -0.3791;
//         e = -4.436;
//         f = 0;
//     }
//     else if (speed == 20) {
//         a = -2E-07;
//         b = 2E-05;
//         c = -0.001;
//         d = 0.0195;
//         e = -1.46;
//         f = 0;
//     }
//     else if (speed == 25) {
//         a = -4E-07;
//         b = 5E-05;
//         c = -0.0026;
//         d = 0.1199;
//         e = -4.86;
//         f = 0;
//     }
//     else if (speed == 30) {
//         a = 1E-07;
//         b = -1E-05;
//         c = -0.0006;
//         d = 0.1309;
//         e = -5.26;
//         f = 0;
//     }
//     else if (speed == 35) {
//         a = -2E-06;
//         b = 0.0002;
//         c = -0.0058;
//         d = 0.1466;
//         e = -4.84;
//         f = 0;
//     }
//     else if (speed == 40) {
//         a = -3E-06;
//         b = 0.0003;
//         c = -0.0086;
//         d = 0.1689;
//         e = -4.24;
//         f = 0;
//     }
//     else if (speed == 45) {
//         a = -3E-06;
//         b = 0.0003;
//         c = -0.0085;
//         d = 0.1493;
//         e = -3.61;
//         f = 0;
//     }
//     else if (speed == 50) {
//         a = -2E-06;
//         b = 0.0002;
//         c = -0.0063;
//         d = 0.1437;
//         e = -3.16;
//         f = 0;
//     }

//     // 计算船舶角度
//     double x2 = x * x;
//     double x3 = x2 * x;
//     double x4 = x3 * x;
//     double x5 = x4 * x;
//     double z = (a * x5 + b * x4 + c * x3 + d * x2 + e * x + f);
//     return z;
// }
//-------------------------------------------------------------------------------

void Controller::tryProcess()
{
    if (auto_mode_ == 0) {
        return;
    }
    std::lock_guard<std::mutex> l(package_lock_);
    if (alg_package_.motor_data.has_value() && alg_package_.imu_data.has_value()) {
        AWARN<<"向自动控制算法传入数据=========================";
        alg_package_.in.mode = auto_mode_;
        // alg_package_.in.current_heading = alg_package_.imu_data.value().heading;
        //---------------------1、船舶当前舵角-------------------------------------
        // 20250822 田鸿宇 新算法用到  船舶  舵角参数
        // 船舶当前舵角
        // alg_package_.in.current_rudder = alg_package_.imu_data.value().current_rudder;
        //---------------------1、船舶当前舵角-------------------------------------
        // float n_s = alg_package_.imu_data.value().north_velocity;
        // float e_s = alg_package_.imu_data.value().east_velocity;
        float speed = alg_package_.imu_data.value().speed;
        alg_package_.in.current_speed = speed;

        //---------------------1、航速最优测试，只变航速-------------------------------------
        // 测试控制算法
        // speed = 10.0; // 模拟的航速  节
        // 20250822 田鸿宇 新算法用到  船舶  舵角参数
        // 船舶当前舵角
        // double current_rudder = -15.0;

        // alg_package_.in.current_speed = speed;
        // alg_package_.in.current_rudder = current_rudder;
        //---------------------1、航速最优测试，只变航速-------------------------------------


        alg_package_.in.pitch_current = alg_package_.imu_data.value().pitch;
        alg_package_.in.heel_current = alg_package_.imu_data.value().roll;
        alg_package_.in.left_current =  thetaToY(alg_package_.motor_data.value()[0].position);
        alg_package_.in.right_current = thetaToY(alg_package_.motor_data.value()[2].position);
        alg_package_.in.max_extension = safe_ext_.getMaxExtensionRatio(speed) * config_info_.max_ext;


        AINFO << "-------speed--------------"<<speed;
        AINFO << "-------max_extension--------------"<<alg_package_.in.max_extension;

        //---------------------2、纵倾/摇最优数据--------------------------------------
        // 20250822 田鸿宇
        // 纵倾/摇最优
        // int speed_int = (int)speed;
        // // 左侧伸出量
        // alg_package_.in.pitch_current = simulation_angle(std::max(alg_package_.in.left_current, alg_package_.in.right_current));
        // //---------------------3、横倾/摇最优数据--------------------------------------
        // // 将最大值赋值给函数
        // if (alg_package_.in.left_current > alg_package_.in.right_current)
        //     alg_package_.in.heel_current  = simulation_angle(alg_package_.in.left_current);
        // else
        //  alg_package_.in.heel_current  = simulation_angle(alg_package_.in.right_current);

        // if (alg_package_.in.heel_current.has_value()) {
        // AINFO << "========横摇数据：" << alg_package_.in.heel_current.value() << "=============";
        // } 
        // else {
        // AINFO << "========横摇数据：无数据=============";
        // }



        AINFO << "send data to alg";
        alg_processor_->add_data(alg_package_.in);
        alg_package_.motor_data.reset();
        alg_package_.imu_data.reset();
    }  
}

void Controller::excuteAlgCmd(const AlgResult& res) {
    // 判断当前模式是否为自动模式及当前自动模式是否和res中自动模式相同，是的话控制 moter_ctrl执行对应命令，不是的话return
    if (res.mode != auto_mode_)
        return;
    //传入截流板伸出量（mm） 转换为电机转动角度
    double theta1 = yToTheta(res.new_left);
    double theta2 = yToTheta(res.new_right);
    ctrl_motor(theta1,theta2);
}

void Controller::tryHandleError() {
    // 电机故障
    bool need_stop = false;
    for (auto& item : monitor_pack_.motor_state) {
        // AWARN <<"错误码"<< item.first << " " << item.second.alarm_code ;
        if (item.second.alarm_code != 101 && item.second.alarm_code != 106
                && item.second.alarm_code != 118 && item.second.alarm_code != 110)  {
            // 严重报警，停止电机
            need_stop = true;
            break;
        }
        else if (item.second.alarm_code == 118) {
            // 控制电机回零
            AERROR << "控制电机回零";
            ctrl_motor(0, 0);
        }
    }

    if (need_stop) {
        for (auto& item : monitor_pack_.motor_state) {
            motor_ctrl_->stopMotor(item.first);
        }
    } else {
        motor_ctrl_->keepRunning();
    }
}

void Controller::ctrl_motor(std::optional<float> left, std::optional<float> right) {
    if (left.has_value()) {
        for(auto& id : config_info_.left_motor) {
            AINFO<<"==============左板控制的电机id："<<id<<"==========转动角度："<<left.value();
            motor_ctrl_->control_motor(MotorController::Command::POSITION_MODE_TARGET, id, left.value());
        }   
    }
    if (right.has_value()) {
        for(auto& id : config_info_.right_motor) {
             AINFO<<"==============右边控制的电机id："<<id<<"==========转动角度："<<right.value();
            motor_ctrl_->control_motor(MotorController::Command::POSITION_MODE_TARGET, id, right.value());
        }   
    }
    
}
// 正函数：由角度θ计算位移y
// 参数theta_deg: 曲柄转角（角度制）
// 返回：截流板伸出量
double Controller::thetaToY(double theta_deg){
     // 角度转弧度
    double theta_rad = theta_deg * DEG_TO_RAD;
    // 计算参数值
    double param = (theta_deg + config_info_.ext2deg.x_min) * DEG_TO_RAD;
    // 应用公式 y = 15√3 + 30sin(θ-60°)
    double result = config_info_.ext2deg.a + config_info_.ext2deg.b * std::sin(param);
    AINFO << "t2y " << theta_deg << " - " << result;
    return result;
}

// 反函数：由位移y反推角度θ
// 参数y: 截流板伸出量
// 返回：曲柄转角（角度制）
double Controller::yToTheta(double y) {
    // 反函数公式：θ = arcsin((y - 15√3)/30) + 60°
    double ratio = (y - config_info_.ext2deg.a) / config_info_.ext2deg.b;
    // 角度结果
    double result_deg = std::asin(ratio) * RAD_TO_DEG - config_info_.ext2deg.x_min;
    if (result_deg < 0) {
        result_deg = 0;
    } else if (result_deg > config_info_.ext2deg.x_max - config_info_.ext2deg.x_min) {
        result_deg = config_info_.ext2deg.x_max - config_info_.ext2deg.x_min;
    }
    AINFO << "y2t " << y << " - " << result_deg;
    return result_deg;
}