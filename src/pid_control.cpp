// =====================================================================
// Author: Tian Hongyu
// Data:2025-12-26
// =====================================================================

#include <glog/logging.h>
#include <sys/stat.h> // 添加此头文件，用于创建目录

#include "log.h" // 确保包含log.h头文件
#include "pid_control.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>

#include <vector>
#include <tuple>

#include <limits>

#include <deque>
#include <numeric>
#include <chrono>
#include <cfloat>

// #include <logging.h>

// 全局常量定义
const float K_output = 3.0f; // 输出增益系数
const float dt = 0.5f;       // 时间间隔（单位：秒）

// =====================================================================
// PID控制器类实现
// =====================================================================

/**
 * @brief PID控制器类，实现基本的PID控制算法
 */
class PIDController
{

public:
    PIDController(float Kp, float Ki, float Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd), prev_error(0), integral(0) {}

    // 添加reset函数
    void reset()
    {
        integral = 0.0f;
        prev_error = 0.0f;
    }

    float compute(float error, float dt)
    {
        // 积分项（抗饱和）
        if (std::abs(error) < 5.0f)
        {
            integral += error * dt;
        }
        else
        {
            integral = 0;
        }

        // 微分项
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        // PID 输出
        float output = Kp * error + Ki * integral + Kd * derivative;
        return output;
    }

private:
    const float Kp;          // 比例系数
    const float Ki;          // 积分系数
    const float Kd;          // 微分系数
    float prev_error = 0.0f; // 上一次误差
    float integral = 0.0f;   // 积分累积值
};

// =====================================================================
// 改进的PID控制器类
// =====================================================================

class ImprovedPIDController
{
public:
    ImprovedPIDController(float Kp, float Ki, float Kd, float max_integral = 10.0f, float dead_zone = 0.5f)
        : Kp(Kp), Ki(Ki), Kd(Kd), max_integral(max_integral), dead_zone(dead_zone),
          prev_error(0), integral(0) {}

    void reset()
    {
        integral = 0.0f;
        prev_error = 0.0f;
    }

    float compute(float error, float dt)
    {
        // 死区处理 - 小误差时不响应
        if (std::abs(error) < dead_zone)
        {
            return 0.0f;
        }

        // 积分项（抗饱和）
        if (std::abs(error) < 5.0f)
        {
            integral += error * dt;
            // 积分限幅
            integral = std::clamp(integral, -max_integral, max_integral);
        }
        else
        {
            integral = 0;
        }

        // 微分项
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        // PID 输出
        float output = Kp * error + Ki * integral + Kd * derivative;

        // 输出限幅
        output = std::clamp(output, -10.0f, 10.0f); // 限制单步变化量

        return output;
    }

private:
    const float Kp;           // 比例系数
    const float Ki;           // 积分系数
    const float Kd;           // 微分系数
    const float max_integral; // 积分上限
    const float dead_zone;    // 死区大小
    float prev_error = 0.0f;  // 上一次误差
    float integral = 0.0f;    // 积分累积值
};

// =====================================================================
// 全局PID控制器实例
// =====================================================================

// 纵倾控制全局参数

// 横倾控制全局参数
const float Kp_heel = 0.5f;
const float Ki_heel = 0.01f;
const float Kd_heel = 0.05f;
PIDController pid_heel(Kp_heel, Ki_heel, Kd_heel);

// 航速最优控制全局参数
const float Kp_speed = 0.5f;
const float Ki_speed = 0.01f;
const float Kd_speed = 0.1f;
// AdaptivePIDSpeed pid_speed(Kp_speed, Ki_speed, Kd_speed, 2.0f, 0.1f);

// 协调转弯控制全局参数
// 小角度
const float Kp_small_turn = 0.4f;
const float Ki_small_turn = 0.01f;
const float Kd_small_turn = 0.1f;
PIDController pid_small_turn(Kp_small_turn, Ki_small_turn, Kd_small_turn);

// 大角度
const float Kp_big_turn = 0.4f;
const float Ki_big_turn = 0.01f;
const float Kd_big_turn = 0.1f;
PIDController pid_big_turn(Kp_big_turn, Ki_big_turn, Kd_big_turn);

// =====================================================================
// 核心控制函数实现
// =====================================================================

// 航速控制类，航速最优/辅助急停，根据实船数据
class SpeedOptimizer
{
public:
    SpeedOptimizer() = default; // 参数移除，构造函数简化

    // 航速最优模式（加速优化）
    std::pair<float, float> compute_optimal_extension(float current_speed,
                                                      float left_current,
                                                      float right_current,
                                                      float max_extension)
    {

        // 下述公式使用的为南盾1航速最优的公式
        float x = current_speed;
        float y = 0.0f;

        if (x < 5)
        {
            y = 0;
        }
        else if (x < 8.7)
        {
            y = 10.81f * x - 54.05f;
        }
        else if (x < 17)
        {
            y = 1.205f * x + 29.52f;
        }
        else if (x <= 50)
        {
            y = 0.052f * x * x - 4.93f * x + 118;
        }
        else
        {
            y = 0;
        }

        return apply_constraints(y, left_current, right_current, max_extension);
    }

    // 辅助急停模式（减速优化）
    std::pair<float, float> compute_brake_extension(float current_speed,
                                                    float left_current,
                                                    float right_current,
                                                    float max_extension,
                                                    float brake_intensity = 1.0f)
    {

        // 下述公式使用的为南盾1航速最优的公式，后续需要根据数据修改
        float x = current_speed;
        float y = 0.0f;

        if (x < 5)
        {
            y = 0;
        }
        else if (x < 8.7)
        {
            y = 10.81f * x - 54.05f;
        }
        else if (x < 17)
        {
            y = 1.205f * x + 29.52f;
        }
        else if (x <= 50)
        {
            y = 0.052f * x * x - 4.93f * x + 118;
        }
        else
        {
            y = 0;
        }

        return apply_constraints(y, left_current, right_current, max_extension);
    }

private:
    // 公共约束处理（防抖动、范围限制）
    std::pair<float, float> apply_constraints(float target_ext,
                                              float left_current,
                                              float right_current,
                                              float max_extension)
    {
        // 物理范围限制
        float new_ext = std::max(0.0f, std::min(target_ext, max_extension));

        // 防抖动处理（基于当前平均值）
        float current_avg_ext = (left_current + right_current) / 2.0f;
        float final_ext = new_ext;

        if (std::abs(new_ext - current_avg_ext) < 1.0f)
        {
            final_ext = current_avg_ext;
        }

        return {final_ext, final_ext};
    }
};

// 协调转弯控制函数 - 基于横倾角的大角度转向
std::pair<float, float> PID_turn_large_control_by_roll(float current_roll_angle,
                                                       float left_current,
                                                       float right_current,
                                                       float max_extension,
                                                       PIDController &pid)
{

    AINFO << "=============================已进入横倾角大角度转向控制程序=====================";
    AINFO << "当前横倾角: " << current_roll_angle << "°";

    // 确定横倾方向（假设右倾为正，左倾为负）
    bool is_right_roll = current_roll_angle > 0;
    AINFO << "横倾方向: " << (is_right_roll ? "右倾" : "左倾");

    // 大角度转向：使用最大伸缩量作为目标
    float target_extension = max_extension;
    AINFO << "大角度横倾，目标伸缩量: " << target_extension << "mm";

    // 根据横倾方向设置控制策略
    float left_target = 0.0f;
    float right_target = 0.0f;

    if (is_right_roll)
    {
        // 右倾：右侧截流板下放，左侧收回为0
        float error = target_extension - right_current;
        float pid_output = pid.compute(error, dt);

        right_target = right_current + error;
        left_target = 0.0f; // 确保左侧收回为0

        AINFO << "右侧PID计算量: " << pid_output;
        AINFO << "右侧目标伸缩量: " << right_target << "mm";
        AINFO << "左侧强制收回为0";
    }
    else
    {
        // 左倾：左侧截流板下放，右侧收回为0
        float error = target_extension - left_current;
        float pid_output = pid.compute(error, dt);

        left_target = left_current + error;
        right_target = 0.0f; // 确保右侧收回为0

        AINFO << "左侧PID计算量: " << pid_output;
        AINFO << "左侧目标伸缩量: " << left_target << "mm";
        AINFO << "右侧强制收回为0";
    }

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(left_target, max_extension));
    float new_right = std::max(0.0f, std::min(right_target, max_extension));

    AINFO << "横倾角大角度转向控制 - 左侧截流板伸缩量: " << new_left << "mm, 右侧截流板伸缩量: " << new_right << "mm";

    return {new_left, new_right};
}

// 协调转弯控制函数 - 基于舵角的大角度转向
std::pair<float, float> PID_turn_large_control(float current_rudder,
                                               float left_current,
                                               float right_current,
                                               float max_extension,
                                               PIDController &pid)
{
    AINFO << "=============================已进入大角度转向控制程序=====================";
    AINFO << "当前舵角: " << current_rudder << "°";

    // 确定转向方向
    bool is_right_turn = current_rudder > 0;
    AINFO << "转向方向: " << (is_right_turn ? "右转" : "左转");

    // 大角度转向：使用最大伸缩量作为目标
    float target_extension = max_extension;
    AINFO << "大角度转向，目标伸缩量: " << target_extension << "mm";

    // 根据转向方向设置控制策略
    float left_target = 0.0f;
    float right_target = 0.0f;

    if (is_right_turn)
    {
        // 右转：右侧截流板下放，左侧收回为0
        float error = target_extension - right_current;

        float pid_output = pid.compute(error, dt);

        right_target = right_current + error;

        left_target = 0.0f; // 确保左侧收回为0

        AINFO << "右侧PID计算量: " << pid_output;
        AINFO << "右侧目标伸缩量: " << right_target << "mm";
        AINFO << "左侧强制收回为0";
    }
    else
    {
        // 左转：左侧截流板下放，右侧收回为0
        float error = target_extension - left_current;

        float pid_output = pid.compute(error, dt);

        left_target = left_current + error;

        right_target = 0.0f; // 确保右侧收回为0

        AINFO << "左侧PID计算量: " << pid_output;
        AINFO << "左侧目标伸缩量: " << left_target << "mm";
        AINFO << "右侧强制收回为0";
    }

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(left_target, max_extension));
    float new_right = std::max(0.0f, std::min(right_target, max_extension));

    AINFO << "大角度转向控制 - 左侧截流板伸缩量: " << new_left << "mm, 右侧截流板伸缩量: " << new_right << "mm";

    return {new_left, new_right};
}

// 横倾最优函数
std::pair<float, float> PID_heel_control(float current_angle,
                                         float target_angle, float left_current,
                                         float right_current,
                                         float max_extension,
                                         PIDController &pid)
{
    float error = current_angle - target_angle; // 计算误差
    float output = pid.compute(error, dt);      // PID计算

    AINFO << "=============================已进入横倾最优程序===================== ";
    AINFO << "PID计算量" << output;
    AINFO << "当前横倾角度" << current_angle;
    AINFO << "目标横倾角度" << target_angle;

    // 反向调整两侧截流板
    float adj_left = left_current - K_output * output;
    float adj_right = right_current + K_output * output;

    AINFO << "左侧原始伸缩量" << left_current;
    AINFO << "右侧原始伸缩量" << right_current;

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(adj_left, max_extension));
    float new_right = std::max(0.0f, std::min(adj_right, max_extension));

    AINFO << "横倾左侧截流板伸缩量：" << new_left;
    AINFO << "横倾右侧截流板伸缩量：" << new_right;

    return {new_left, new_right};
}

// =====================================================================
// 主接口函数
// =====================================================================

PID_Output PID_parameter_transfer(const PID_Input &input)
{
    // 使用log.h封装的宏替代原生glog
    AINFO << "接收到的截流板最大伸缩量: " << input.max_extension << "mm";
    AINFO << "左侧当前: " << input.left_current
          << "mm, 右侧当前: " << input.right_current << "mm";

    float new_left = input.left_current;
    float new_right = input.right_current;
    int error_code = 0;    // 存储错误代码（数字），0=成功，非0=错误
    std::string error_msg; // 存储错误描述（文字）

    // ============ 所有模式通用的数据校验 ============
    // 先使用has_value()检查是否有值，然后再用value()来获取值
    if (!input.current_speed.has_value())
    {
        error_msg = "需要提供当前航速";
        AERROR << error_msg; // 使用AERROR宏

        // 构建错误返回结果
        PID_Output output;
        output.new_left = new_left;
        output.new_right = new_right;
        output.error_code = 10; // 通用错误码，表示速度未提供
        output.error_msg = error_msg;
        return output;
    }

    float current_speed = input.current_speed.value();

    if (current_speed < 0)
    {
        error_msg = "航速必须大于0";
        AERROR << error_msg; // 使用AERROR宏

        // 构建错误返回结果
        PID_Output output;
        output.new_left = new_left;
        output.new_right = new_right;
        output.error_code = 20; // 通用错误码，表示速度小于0
        output.error_msg = error_msg;
        return output;
    }

    // 伸缩量校验
    if (input.left_current < -1 || input.right_current < -1)
    {
        error_msg = "当前伸缩量不能为较大负数";
        AERROR << error_msg;

        // 构建错误返回结果
        PID_Output output;
        output.new_left = new_left;
        output.new_right = new_right;
        output.error_code = 30; // 通用错误码，表示伸缩量异常
        output.error_msg = error_msg;
        return output;
    }

    // 增加航速极大值的限制
    const float MAX_REASONABLE_SPEED = 100.0f; // 最大航速阈值设置（可根据实际情况修改）
    if (current_speed > MAX_REASONABLE_SPEED)
    {
        error_msg = "航速超过合理范围";
        AERROR << error_msg;

        // 构建错误返回结果
        PID_Output output;
        output.new_left = new_left;
        output.new_right = new_right;
        output.error_code = 40; // 通用错误码，表示速度超出范围
        output.error_msg = error_msg;
        return output;
    }

    switch (input.mode)
    {

        // 日志打印使用说明
        // AINFO << "信息";
        // AWARN << "警告";
        // AERROR << "错误";
        // AFATAL << "致命错误";

    case 1:
    {                                                     // 自动模式
        AINFO << "[自动模式] 保持当前状态，尚未完成开发"; // 使用AINFO宏
        break;
    }

        // [暂时空置] （原暂定节能）
    case 2:
    {
        AINFO << "[暂时空置] （原暂定节能）"; // 使用AINFO宏
        break;
    }

    // 航速最优模式(同步实现了纵倾最优)
    case 31:
    {
        AINFO << "=============================切换为航速最优/纵倾最优模式========================";

        AINFO << "当前接收到的速度" << current_speed
              << "截流板当前阈值" << input.max_extension;

        // 对象创建
        SpeedOptimizer optimizer;
        auto result = optimizer.compute_optimal_extension(
            current_speed,       // 当前航速
            input.left_current,  // 左舷当前伸缩量
            input.right_current, // 右舷当前伸缩量
            input.max_extension  // 最大伸缩量限制
        );

        new_left = result.first;
        new_right = result.second;

        AINFO << "[航速最优模式] 目标伸缩量: 左=" << new_left // 使用AINFO宏
              << "mm, 右=" << new_right << "mm";
        break;
    }

    // 辅助急停模式
    case 32:
    {
        AINFO << "=============================切换为辅助急停模式========================";

        AINFO << "当前接收到的速度" << current_speed
              << "截流板当前阈值" << input.max_extension;

        // 对象创建
        SpeedOptimizer optimizer;
        auto result = optimizer.compute_brake_extension(
            current_speed,       // 当前航速
            input.left_current,  // 左舷当前伸缩量
            input.right_current, // 右舷当前伸缩量
            input.max_extension  // 最大伸缩量限制
        );

        new_left = result.first;
        new_right = result.second;

        AINFO << "[辅助急停模式] 目标伸缩量: 左=" << new_left // 使用AINFO宏
              << "mm, 右=" << new_right << "mm";
        break;
    }

    // 协调转弯控制模式（第一种判断模式：舵角判断）
    case 33:
    {
        AINFO << "=============================切换为协调转弯模式（基于舵角判断）========================";

        float current_speed = input.current_speed.value();
        AINFO << "当前接收到的速度" << current_speed
              << "截流板当前阈值" << input.max_extension;

        // 检查舵角数据是否存在
        if (!input.current_rudder.has_value())
        {
            error_code = 330;
            error_msg = "协调转弯模式（舵角判断）需要提供当前舵角";
            AERROR << error_msg;
            break;
        }

        float current_rudder = input.current_rudder.value();

        // 检查舵角是否在有效范围内 (-35° 到 +35°)
        if (current_rudder < -35.0f || current_rudder > 35.0f)
        {
            error_code = 331;
            error_msg = "舵角必须在-35°到+35°范围内";
            AERROR << error_msg;
            break;
        }

        // 舵角为0时表示直航，不进行协调转弯
        if (std::abs(current_rudder) < 1.0f)
        {
            AINFO << "舵角(" << current_rudder << "°) ≈ 0°，直航状态，截流板全部收回";
            new_left = 0.0f;
            new_right = 0.0f;
            break;
        }

        std::pair<float, float> result;
        float rudder_magnitude = std::abs(current_rudder);

        // 根据舵角大小选择控制策略
        if (rudder_magnitude < 3.0f)
        {
            // 舵角小于3°：全部收回
            AINFO << "舵角(" << current_rudder << "°) < 3°，截流板全部收回";
            result = {0.0f, 0.0f};
        }
        else if (rudder_magnitude >= 3.0f && rudder_magnitude <= 5.0f)
        {
            // 舵角在上述范围内：保持当前状态
            AINFO << "舵角(" << current_rudder << "°) 在3°~5°范围内，保持当前状态";
            result = {input.left_current, input.right_current};
        }
        else
        {
            // 舵角大于5°：进行转向控制
            AINFO << "舵角(" << current_rudder << "°) > 5°，进入转向控制";
            result = PID_turn_large_control_by_roll(current_roll_angle,
                                                    input.left_current,
                                                    input.right_current,
                                                    input.max_extension,
                                                    pid_big_turn);
        }

        new_left = result.first;
        new_right = result.second;

        // 检查变化量是否小于1mm（满足条件则保持当前值）
        if (std::abs(new_left - input.left_current) < 1.0f)
        {
            AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << input.left_current << "mm";
            new_left = input.left_current;
        }

        if (std::abs(new_right - input.right_current) < 1.0f)
        {
            AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << input.right_current << "mm";
            new_right = input.right_current;
        }

        AINFO << "[协调转弯] 最终伸缩量: 左=" << new_left << "mm, 右=" << new_right << "mm";
        break;
    }

    // // 协调转弯控制模式（第二种判断模式：横倾角判断）
    // case 33:
    // {
    //     AINFO << "=============================切换为协调转弯模式（基于横摇角判断）========================";

    //     float current_speed = input.current_speed.value();
    //     AINFO << "当前接收到的速度" << current_speed
    //           << "截流板当前阈值" << input.max_extension;

    //     // 检查横摇角数据是否存在
    //     if (!input.heel_current.has_value())
    //     {
    //         error_code = 330;
    //         error_msg = "协调转弯模式（横摇角判断）需要提供当前横摇角";
    //         AERROR << error_msg;
    //         break;
    //     }

    //     // 获取当前横倾角
    //     float current_roll_angle = input.heel_current.value();
    //     float roll_magnitude = std::abs(current_roll_angle);
    //     std::pair<float, float> result;

    //     // 根据横倾角大小选择控制策略
    //     if (roll_magnitude < 3.0f)
    //     {
    //         // 横倾角小于3°：全部收回
    //         AINFO << "横倾角(" << current_roll_angle << "°) < 3°，截流板全部收回";
    //         result = {0.0f, 0.0f};
    //     }
    //     else if (roll_magnitude >= 3.0f && roll_magnitude <= 5.0f)
    //     {
    //         // 横倾角在上述范围内：保持当前状态
    //         AINFO << "横倾角(" << current_roll_angle << "°) 在范围内，保持当前状态";
    //         result = {input.left_current, input.right_current};
    //     }
    //     else
    //     {
    //         // 横倾角大于5°：进行转向控制
    //         AINFO << "横倾角(" << current_roll_angle << "°) > 5°，进入转向控制";
    //         result = PID_turn_large_control_by_roll(current_roll_angle,
    //                                                 input.left_current,
    //                                                 input.right_current,
    //                                                 input.max_extension,
    //                                                 pid_big_turn);
    //     }

    //     new_left = result.first;
    //     new_right = result.second;

    //     // 检查变化量是否小于1mm（满足条件则保持当前值）
    //     if (std::abs(new_left - input.left_current) < 1.0f)
    //     {
    //         AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << input.left_current << "mm";
    //         new_left = input.left_current;
    //     }

    //     if (std::abs(new_right - input.right_current) < 1.0f)
    //     {
    //         AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << input.right_current << "mm";
    //         new_right = input.right_current;
    //     }

    //     AINFO << "[协调转弯] 最终伸缩量: 左=" << new_left << "mm, 右=" << new_right << "mm";
    //     break;
    // }

    // 纵倾最优控制（暂时空置）
    case 34:
    {
        AINFO << "[暂时空置] （该功能合并至航速最优模式）"; // 使用AINFO宏
        break;
    }

    // 横倾控制模式 备份
    case 35:
    {
        if (!input.heel_current.has_value())
        {
            error_code = 350;
            error_msg = "横倾模式需要提供当前横倾角度";
            AERROR << error_msg; // 使用AERROR宏
            break;
        }
        float current_speed = input.current_speed.value();
        AINFO << "当前接收到的速度" << current_speed << "截流板当前阈值" << input.max_extension;

        float current_angle = input.heel_current.value();
        float target_angle = input.heel_target.value_or(0.0f);

        AINFO << "当前横倾角度: " << current_angle << "°"; // 使用AINFO宏

        auto result = PID_heel_control(current_angle, target_angle,
                                       input.left_current, input.right_current,
                                       input.max_extension, pid_heel);

        new_left = result.first;
        new_right = result.second;

        // 检查变化量是否小于1mm（满足条件则保持当前值）
        if (std::abs(new_left - input.left_current) < 1.0f)
        {
            AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << input.left_current << "mm";
            new_left = input.left_current;
        }

        if (std::abs(new_right - input.right_current) < 1.0f)
        {
            AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << input.right_current << "mm";
            new_right = input.right_current;
        }

        AINFO << "[横倾模式] 目标伸缩量: 左=" << new_left // 使用AINFO宏
              << "mm, 右=" << new_right << "mm";
        break;
    }

    default:
        error_code = 106;
        error_msg = "无效控制模式: " + std::to_string(input.mode);
        AERROR << error_msg; // 使用AERROR宏
    }

    AINFO << "-------------------------------------------------------"; // 使用AINFO宏

    // 修改返回语句
    PID_Output result;
    result.mode = input.mode;
    result.new_left = new_left;
    result.new_right = new_right;
    result.error_code = error_code;
    result.error_msg = error_msg;
    return result;
}
