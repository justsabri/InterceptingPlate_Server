// =====================================================================
// Author: Tian Hongyu
// Data:2026-01-05
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
        // 物理范围限制：将目标伸缩量限制在 [0, max_extension] 范围内，
        // 防止传入非法负值或超过机械极限的值导致越界。
        float new_ext = std::max(0.0f, std::min(target_ext, max_extension));

        // 防抖动处理（基于当前平均值）：
        // 计算左右当前伸缩量的平均值，作为当前系统的基线值，
        // 之所以采用平均值，是因为左右两个执行机构通常应协同工作，
        // 用平均值可以平滑单侧噪声或瞬时抖动带来的影响。
        float current_avg_ext = (left_current + right_current) / 2.0f;

        // 默认最终伸缩量为经过物理限制后的目标值
        float final_ext = new_ext;

        // 若目标值与当前平均值的差异小于阈值（1mm），则视为抖动或微小调整，
        // 此时保持当前平均值不变以避免执行机构频繁小幅动作。
        if (std::abs(new_ext - current_avg_ext) < 1.0f)
        {
            final_ext = current_avg_ext;
        }

        // 目前返回左右相同的伸缩量对（对称控制）。若后续需要非对称动作，
        // 可在此处修改为返回不同的左右值。
        return {final_ext, final_ext};
    }
};

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

    // 根据转向方向设置控制模式
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

    // 防抖动处理：若变化量小于1mm，则保持当前值
    if (std::abs(new_left - left_current) < 1.0f)
    {
        AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << left_current << "mm";
        new_left = left_current;
    }

    if (std::abs(new_right - right_current) < 1.0f)
    {
        AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << right_current << "mm";
        new_right = right_current;
    }

    AINFO << "大角度转向控制 - 左侧截流板伸缩量: " << new_left << "mm, "
          << "右侧截流板伸缩量: " << new_right << "mm";

    return {new_left, new_right};
}

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

    // 根据横倾方向设置控制模式
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

    // 防抖动处理：若变化量小于1mm，则保持当前值
    if (std::abs(new_left - left_current) < 1.0f)
    {
        AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << left_current << "mm";
        new_left = left_current;
    }

    if (std::abs(new_right - right_current) < 1.0f)
    {
        AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << right_current << "mm";
        new_right = right_current;
    }

    AINFO << "横倾角大角度转向控制 - 左侧截流板伸缩量: " << new_left << "mm, 右侧截流板伸缩量: " << new_right << "mm";

    return {new_left, new_right};
}

// 横倾最优函数（接收完整输入并包含前置的日志打印与后置1mm保持逻辑，便于复用）
std::pair<float, float> PID_heel_control(const PID_Input &input, PIDController &pid)
{
    // 使用 input 中的值，调用者需保证已对必要字段进行存在性校验
    float current_speed = input.current_speed.value_or(0.0f);
    AINFO << "=============================已进入横倾最优程序===================== ";
    AINFO << "当前接收到的速度" << current_speed << "节"
          << "截流板当前阈值" << input.max_extension << "mm";

    float current_angle = input.heel_current.value_or(0.0f);
    float target_angle = input.heel_target.value_or(0.0f);

    float error = current_angle - target_angle; // 计算误差
    float output = pid.compute(error, dt);      // PID计算

    AINFO << "PID计算量" << output;
    AINFO << "当前横倾角度" << current_angle;
    AINFO << "目标横倾角度" << target_angle;

    // 反向调整两侧截流板
    float adj_left = input.left_current - K_output * output;
    float adj_right = input.right_current + K_output * output;

    AINFO << "左侧原始伸缩量" << input.left_current;
    AINFO << "右侧原始伸缩量" << input.right_current;

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(adj_left, input.max_extension));
    float new_right = std::max(0.0f, std::min(adj_right, input.max_extension));

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

    AINFO << "横倾左侧截流板伸缩量：" << new_left;
    AINFO << "横倾右侧截流板伸缩量：" << new_right;

    AINFO << "[横倾模式] 目标伸缩量: 左=" << new_left << "mm, " << "右=" << new_right << "mm";

    return {new_left, new_right};
}

// 协调转弯通用函数：coord_turn_select=1 使用舵角判断，=2 使用横摇/横倾角判断
std::pair<float, float> PID_coord_turn_control(const PID_Input &input, // 输入参数，为常量，不可修改
                                               int coord_turn_select,  // 判断模式选择
                                               int &error_code,        // 错误代码引用，为非常量，可修改
                                               std::string &error_msg) // 错误信息引用，为非常量，可修改
{
    float new_left = input.left_current;
    float new_right = input.right_current;

    if (coord_turn_select == 1)
    {
        AINFO << "=============================协同转弯（舵角判断）调用函数========================";

        // 如果未提供舵角数据 -> 设置错误并返回当前值
        if (!input.current_rudder.has_value())
        {
            error_code = 330;
            error_msg = "协调转弯模式（舵角判断）需要提供当前舵角";
            AINFO << "协调转弯模式（舵角判断）需要提供当前舵角";
            return {new_left, new_right};
        }

        // 读取舵角值并进行范围检查
        float current_rudder = input.current_rudder.value();
        // 如果舵角超出物理或安全范围 -> 设置错误并返回当前值
        if (current_rudder < -35.0f || current_rudder > 35.0f)
        {
            error_code = 331;
            error_msg = "舵角必须在-35°到+35°范围内";
            return {new_left, new_right};
        }

        // 如果舵角接近 0 -> 视为直航，全部收回
        if (std::abs(current_rudder) < 1.0f)
        {
            AINFO << "舵角(" << current_rudder << "°) ≈ 0°，直航状态，截流板全部收回";
            return {0.0f, 0.0f};
        }

        // 计算舵角绝对值以决定分支
        std::pair<float, float> result;
        float rudder_magnitude = std::abs(current_rudder);

        // 舵角小于 3° -> 全部收回（轻微转向不处理）
        if (rudder_magnitude < 3.0f)
        {
            AINFO << "舵角(" << current_rudder << "°) < 3°，截流板全部收回";
            result = {0.0f, 0.0f};
        }
        // 舵角在 3°~5° -> 轻微转向，保持当前伸缩量
        else if (rudder_magnitude >= 3.0f && rudder_magnitude <= 5.0f)
        {
            AINFO << "舵角(" << current_rudder << "°) 在3°~5°范围内，保持当前状态";
            result = {input.left_current, input.right_current};
        }
        // 舵角大于 5° -> 进入大角度转向控制逻辑
        else
        {
            AINFO << "舵角(" << current_rudder << "°) > 5°，进入转向控制";
            result = PID_turn_large_control(current_rudder,
                                            input.left_current,
                                            input.right_current,
                                            input.max_extension,
                                            pid_big_turn);
        }

        // 返回根据舵角计算的结果
        return result;
    }
    else if (coord_turn_select == 2)
    {
        AINFO << "=============================协同转弯（横摇/横倾判断）调用函数========================";
        // 如果未提供横倾/横摇数据 -> 设置错误并返回当前值
        if (!input.heel_current.has_value())
        {
            error_code = 330;
            error_msg = "协调转弯模式（横摇角判断）需要提供当前横摇角";
            AINFO << "协调转弯模式（横摇角判断）需要提供当前横摇角";
            return {new_left, new_right};
        }

        // 读取横倾角并判断幅值
        float current_roll_angle = input.heel_current.value();
        float roll_magnitude = std::abs(current_roll_angle);
        std::pair<float, float> result;

        // 横倾角小于 3° -> 全部收回
        if (roll_magnitude < 3.0f)
        {
            AINFO << "横倾角(" << current_roll_angle << "°) < 3°，截流板全部收回";
            result = {0.0f, 0.0f};
        }
        // 横倾角在 3°~5° -> 维持当前状态
        else if (roll_magnitude >= 3.0f && roll_magnitude <= 5.0f)
        {
            AINFO << "横倾角(" << current_roll_angle << "°) 在范围内，保持当前状态";
            result = {input.left_current, input.right_current};
        }
        // 横倾角大于 5° -> 使用横倾角驱动的大角度转向控制
        else
        {
            AINFO << "横倾角(" << current_roll_angle << "°) > 5°，进入转向控制";
            result = PID_turn_large_control_by_roll(current_roll_angle,
                                                    input.left_current,
                                                    input.right_current,
                                                    input.max_extension,
                                                    pid_big_turn);
        }

        // 返回根据横倾角计算的结果
        return result;
    }

    error_code = 333;
    error_msg = "coord_turn_select 参数无效（应为1或2）";
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
    if (input.left_current < -2 || input.right_current < -2)
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
        // AINFO << "信息"，绿色，仅仅记录;
        // AWARN << "警告"，黄色，记录并提醒可能存在的问题;
        // AERROR << "错误"，红色，记录并提醒存在的问题，需要处理;
        // AFATAL << "致命错误"，红色加粗，程序终止，记录并提醒存在的严重问题，程序无法继续运行;

    // 自动模式
    case 1:
    {
        AINFO << "=============================切换为自动模式========================";

        // 输出结果仍然使用 mode=1，表明调用方传入的是自动模式（而不是把子模式替回去）
        // 内部我们根据实时数据选择合适的子策略（航速最优/辅助急停/协调转弯/横倾最优），
        // 但不改变外部传入的 mode 字段（保持外部可见为自动）。

        // ========== 静态变量用于状态保持 ==========
        // static 关键字在函数作用域声明变量后，该变量在函数多次调用间保持其值不被销毁。
        // 这里用来记住上一次自动选择的逻辑状态，避免每次都清空历史数据或状态。
        static int current_logic_state = 31; // 31:航速最优, 32:辅助急停, 33:协调转弯, 35:横倾最优

        // speed_history 存放最近若干个时间点的主机转速，用于检测短时间内转速是否快速下降（急停）
        // 说明：
        // - std::deque 是双端队列，支持从头尾高效插入和删除；适合用来维护时间窗口的数据。
        // - std::pair<time_t,float> 表示一条记录，first=time_t（时间戳），second=float（对应RPM）。
        static std::deque<std::pair<time_t, float>> speed_history;

        // ========== 参数检查（必须字段） ==========
        // has_value() 用于检查 std::optional 是否包含值；若没有值我们不能继续自动判别。
        if (!input.current_RPM.has_value() || !input.current_rudder.has_value() ||
            !input.heel_current.has_value())
        {
            // 当必要字段缺失时，设置错误信息并退出当前 case（保留现有 new_left/new_right）
            error_msg = "自动模式需要提供主机转速、舵角和横倾角度";
            AERROR << error_msg;
            error_code = 101;
            break; // 终止 case 1，返回默认或之前的截流板值
        }

        // 从 std::optional 安全取值：value() 在 has_value() 为 true 时返回内部值
        float current_RPM = input.current_RPM.value();
        float current_rudder = input.current_rudder.value();
        float heel_current = input.heel_current.value();

        // ========== 辅助急停检测（优先级最高） ==========
        // 思路：维护最近 10 秒的 RPM 记录，判断短时间（≤5s）内是否从高 RPM 跌至低 RPM，且降速足够快
        // 如果满足条件则切换到辅助急停（current_logic_state = 32）以保证安全。

        // time(nullptr) 返回当前时间（time_t），用于构建时间序列
        time_t current_time = time(nullptr);
        // 将当前时间和 RPM 压入队列尾部
        speed_history.push_back(std::make_pair(current_time, current_RPM));

        // 清理早于 10 秒前的记录，保持队列只包含最近 10 秒的数据
        while (!speed_history.empty() &&
               (current_time - speed_history.front().first) > 10)
        {
            // pop_front() 从队首移除最早的记录
            speed_history.pop_front();
        }

        // 默认未检测到急停
        bool emergency_brake_detected = false;

        // 只有当队列中至少有2条记录时，才有意义计算时间差和速度变化
        if (speed_history.size() >= 2)
        {
            // 使用引用避免拷贝；front() 返回队首元素，back() 返回队尾元素
            auto &oldest = speed_history.front();
            auto &newest = speed_history.back();

            // difftime 返回两个 time_t 之间的差（以秒为单位，返回 double）
            float time_diff = difftime(newest.first, oldest.first);

            // 限制检测窗口：只关注跨度不超过 5 秒且大于 0 的情况
            if (time_diff <= 5.0 && time_diff > 0)
            {
                // speed_diff 表示在该时间窗口内 RPM 的下降量
                float speed_diff = oldest.second - newest.second;
                // avg_descent_rate 为平均每秒下降多少 RPM（可能为正值表示下降）
                float avg_descent_rate = speed_diff / time_diff;

                // 判定条件（示例阈值，可根据实际船舶数据调整）：
                // - 起始 RPM 较高（>=1800）且最新 RPM 已非常低（<=800），表示明显降速；
                // - 平均降速速率 >= 100 rpm/s（阈值可灵活调整）
                if (oldest.second >= 1800.0f &&
                    newest.second <= 800.0f &&
                    avg_descent_rate >= 100.0f)
                {
                    // 标记检测到急停，切换内部逻辑为辅助急停
                    emergency_brake_detected = true;
                    current_logic_state = 32; // 32 表示辅助急停
                    AINFO << "[自动模式] 检测到急停条件，使用辅助急停模式";
                }
            }
        }

        // ========== 如果不是急停，按优先级判断其他模式 ==========
        if (!emergency_brake_detected)
        {
            // 优先级1: 协调转弯 (舵角绝对值≥10)
            if (fabs(current_rudder) >= 10.0f)
            {
                current_logic_state = 33; // 协调转弯
                AINFO << "[自动模式] 舵角" << current_rudder << "°，使用协调转弯模式";
            }
            // 优先级2: 舵角在(5,10)之间保持当前状态
            else if (fabs(current_rudder) > 5.0f && fabs(current_rudder) < 10.0f)
            {
                AINFO << "[自动模式] 舵角" << current_rudder << "°，保持当前模式";
                // current_logic_state 不变
            }
            // 优先级3: 舵角≤5时判断横摇
            else
            {
                // 横摇绝对值≥5，使用横倾最优模式
                if (fabs(heel_current) >= 5.0f)
                {
                    current_logic_state = 35; // 横倾最优
                    AINFO << "[自动模式] 横摇" << heel_current << "°，使用横倾最优模式";
                }
                // 横摇在(3,5)之间保持当前状态
                else if (fabs(heel_current) > 3.0f && fabs(heel_current) < 5.0f)
                {
                    AINFO << "[自动模式] 横摇" << heel_current << "°，保持当前模式";
                    // current_logic_state 不变
                }
                // 横摇≤3，使用航速最优模式
                else
                {
                    current_logic_state = 31; // 航速最优
                    AINFO << "[自动模式] 横摇" << heel_current << "°，使用航速最优模式";
                }
            }
        }

        // ========== 根据选择的逻辑状态执行相应的控制模式 ==========
        // 直接复制case 31-35的代码，但只在内部使用
        switch (current_logic_state)
        {
        case 31: // 航速最优模式 (对应原case 31)
        {
            AINFO << "[自动模式-航速最优模式]";

            AINFO << "当前接收到的速度" << current_speed
                  << "截流板当前阈值" << input.max_extension;

            SpeedOptimizer optimizer;
            auto result = optimizer.compute_optimal_extension(
                current_speed,
                input.left_current,
                input.right_current,
                input.max_extension);

            new_left = result.first;
            new_right = result.second;

            AINFO << "[自动模式-航速最优] 目标伸缩量: 左=" << new_left
                  << "mm, 右=" << new_right << "mm";
            break;
        }

        case 32: // 辅助急停模式 (对应原case 32)
        {
            AINFO << "[自动模式-辅助急停模式]";

            AINFO << "当前接收到的速度" << current_speed
                  << "截流板当前阈值" << input.max_extension;

            SpeedOptimizer optimizer;
            auto result = optimizer.compute_brake_extension(
                current_speed,
                input.left_current,
                input.right_current,
                input.max_extension);

            new_left = result.first;
            new_right = result.second;

            AINFO << "[自动模式-辅助急停] 目标伸缩量: 左=" << new_left
                  << "mm, 右=" << new_right << "mm";
            break;
        }

        case 33: // 协调转弯模式 (对应原case 33)
        {
            AINFO << "[自动模式-协调转弯模式]";

            // 使用舵角判断
            int coord_turn_select = 1;

            auto result = PID_coord_turn_control(
                input,
                coord_turn_select,
                error_code,
                error_msg);

            new_left = result.first;
            new_right = result.second;

            if (error_code != 0)
            {
                AERROR << error_msg;
            }

            AINFO << "[自动模式-协调转弯] 最终伸缩量: 左=" << new_left << "mm, "
                  << "右=" << new_right << "mm";
            break;
        }

        case 35: // 横倾最优模式 (对应原case 35)
        {
            AINFO << "[自动模式-横倾最优模式]";

            if (!input.heel_current.has_value())
            {
                error_code = 350;
                error_msg = "横倾最优模式需要提供当前横倾角度";
                AERROR << error_msg;
                break;
            }

            auto result = PID_heel_control(input, pid_heel);
            new_left = result.first;
            new_right = result.second;
            break;
        }
        }

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

    // 协调转弯模式
    case 33:
    {
        // 修改此变量的值以切换判断模式：1 使用舵角判断，2 使用横摇角判断
        int coord_turn_select = 1; // <-- 修改为 1 或 2 即可切换（仅修改此行，无需改动头文件）

        // 协调转弯通用函数调用，result接收主要返回值，error_code/error_msg通过引用即可被修改
        // 在C++中，函数通常通过返回值来返回主要结果，但有时也需要返回额外的信息（比如错误码和错误信息）。
        // 如果通过返回值返回错误码，那么主要结果（这里是一对浮点数）就需要通过引用或指针参数来返回，这会让函数调用看起来不那么直观。
        auto result = PID_coord_turn_control(input, coord_turn_select, error_code, error_msg);

        new_left = result.first;
        new_right = result.second;

        // 异常值判断
        if (error_code != 0)
        {
            AERROR << error_msg;
        }

        AINFO << "[协调转弯] 最终伸缩量: 左=" << new_left << "mm, "
              << "右=" << new_right << "mm";
        break;
    }

    // 纵倾最优控制（暂时空置）
    case 34:
    {
        AINFO << "[暂时空置] （该功能合并至航速最优模式）"; // 使用AINFO宏
        break;
    }

    // 横倾最优模式 备份
    case 35:
    {
        if (!input.heel_current.has_value())
        {
            error_code = 350;
            error_msg = "横倾最优模式需要提供当前横倾/摇角度";
            AERROR << error_msg; // 使用AERROR宏
            break;
        }
        auto result = PID_heel_control(input, pid_heel);

        new_left = result.first;
        new_right = result.second;
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
