#pragma once
#include "event_bus.h"
#include <string>
#include <mutex>
#include <any>
#include <nlohmann/json.hpp>
#include "Thread_Pool.h"
#include "alg_processor.h"
#include "data_struct.h"
#include <optional>
#include "motor_controller.h"
#include "monitor_system.h"
#include <cmath>
#include <stdexcept>

// 常量定义
const double SQRT3 = 1.73205080757; // √3的近似值
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

using json = nlohmann::json; // 别名简化json类型
class Controller
{
public:
    Controller(EventBus &bus);
    void start();
    void sendDataToClient(Data_Type type, void *data);
    void handle_message(const json &j);
    double thetaToY(double theta_deg);
    double yToTheta(double y);

private:
    class SafeExtension
    {
    public:
        SafeExtension() {};
        ~SafeExtension() {};
        float getMaxExtensionRatio(float speed /*kn*/)
        {
            float ratio = 1.0;
            if (speed <= 20)
            {
                return ratio;
            }
            else if (speed <= 40)
            {
                ratio = 2.2 - 0.06 * speed;
                //-----------会存在小于0------------------
                if (ratio < 0.0)
                {
                    ratio = 0.0;
                }
            }
            else
            {
                ratio = 0;
            }
            return ratio;
        }
    };

    void convertStructToJson(Data_Type type, void *data, json &j);
    void tryProcess();
    void excuteAlgCmd(const AlgResult &res);
    void ctrl_motor(std::optional<float> left, std::optional<float> right);

    typedef struct
    {
        std::optional<std::map<int, MotorData>> motor_data;
        std::optional<ImuData> imu_data;
        AlgInput in;
    } Alg_Data_Package;

    typedef struct {
        float a;
        float b;
        float x_min;
        float x_max;
    } ext2deg_formula1;

    typedef struct
    {
        int motor_num;
        std::vector<int> left_motor;
        std::vector<int> right_motor;
        float max_ext;
        ext2deg_formula1 ext2deg;
    } Config_Info;

    EventBus &event_bus_;
    ThreadPool thread_pool_;

    std::unique_ptr<AlgProcessor> alg_processor_;
    std::thread alg_worker_;
    Alg_Data_Package alg_package_;
    std::mutex package_lock_;
    int auto_mode_;

    std::function<void(ImuData data)> imu_data_cb;
    std::function<void(std::map<int, MotorData> data)> motor_data_cb;

    std::unique_ptr<MotorController> motor_ctrl_;
    Config_Info config_info_;

    std::unique_ptr<MonitoringSystem> monitor_system_;
    DataPack monitor_pack_;

    SafeExtension safe_ext_;
};
