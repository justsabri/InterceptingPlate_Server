// =====================================================================
// Author: Tian Hongyu
// Data:2026-04-20
// Version:Base DF A（通用）
// Description: 当前辅助制动使用的是基于7.5m的实船数据

// 2026.4.19更新
// 1.左右急停判断错误，左右主机调用该函数时共享同一份状态变量参数，导致一侧进入急停后，另一侧也被误判为急停。
// 2.航速降低时，未能正常收回的问题解决，主函数初始数据判断时，若未通过判断则将伸缩量置0。
// 
// 2026.4.20更新
// 1.对横倾的伸缩量进行了优化，引入了横倾角的缩放因子*航速缩放因子，进行伸缩量的调节。
//
// 2026.4.22更新
// 1. 针对左右主机急停判断函数：使用独立的静态变量，加入right或left进行区分;
// 2. 修复了时间计算Bug；
// 3. 修改了退出条件B：由25秒改成20秒
// 4. 增加了退出条件C：5 秒未降到 800 转退出急停
// 5. 在Case中引入了static静态变量，用于锁存急停状态，防止二次收回和左右串扰问题；
// 6. 明确一下主函数中伸缩量赋值逻辑，传感器坏了/数据无效 → 返回 0（安全收回），程序正常、只是没匹配模式 → 保持当前位置；
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
const float dt = 0.1f;       // 时间间隔（单位：秒）


// =====================================================================
// 相关PID控制器类实现
// =====================================================================

// 经典PID控制器类
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
const float Kp_heel = 0.8f;
const float Ki_heel = 0.05f;
const float Kd_heel = 0.01f;
PIDController pid_heel(Kp_heel, Ki_heel, Kd_heel);

// 航速提升（原航速-最优）控制全局参数
const float Kp_speed = 0.8f;
const float Ki_speed = 0.05f;
const float Kd_speed = 0.01f;
// AdaptivePIDSpeed pid_speed(Kp_speed, Ki_speed, Kd_speed, 2.0f, 0.1f);

// 回转改善（原协调-转弯）控制全局参数
// 大角度
const float Kp_big_turn = 0.4f;
const float Ki_big_turn = 0.05f;
const float Kd_big_turn = 0.01f;
PIDController pid_big_turn(Kp_big_turn, Ki_big_turn, Kd_big_turn);

// =============================================================================
// 船舶横摇模糊PD控制器 (专用于有限行程截流板，去除积分项防止饱和)
// =============================================================================
enum FuzzySet
{
    NB,
    NM,
    NS,
    ZO,
    PS,
    PM,
    PB
};

struct MFParams
{
    float a, b, c;
};

// [TUNING] 隶属度函数 (与之前相同，无需修改)
MFParams mf_E[7] = {
    {-6.0f, -6.0f, -4.0f}, {-6.0f, -4.0f, -2.0f}, {-4.0f, -2.0f, 0.0f}, {-2.0f, 0.0f, 2.0f}, {0.0f, 2.0f, 4.0f}, {2.0f, 4.0f, 6.0f}, {4.0f, 6.0f, 6.0f}};
MFParams mf_EC[7] = {
    {-3.0f, -3.0f, -2.0f}, {-3.0f, -2.0f, -1.0f}, {-2.0f, -1.0f, 0.0f}, {-1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 2.0f}, {1.0f, 2.0f, 3.0f}, {2.0f, 3.0f, 3.0f}};
MFParams mf_OUT[7] = {
    {-3.0f, -3.0f, -2.0f}, {-3.0f, -2.0f, -1.0f}, {-2.0f, -1.0f, 0.0f}, {-1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 2.0f}, {1.0f, 2.0f, 3.0f}, {2.0f, 3.0f, 3.0f}};

// [MODIFIED] 仅保留 ΔKp 和 ΔKd 规则表 (Ki 已移除)
// ΔKp 规则表 (船舶横摇专用：误差大且同向时强力拉回)
FuzzySet ruleKp[7][7] = {
    // NB    NM    NS    ZO    PS    PM    PB
    {PB, PB, PM, PM, PS, ZO, ZO}, // NB
    {PB, PB, PM, PS, PS, ZO, NS}, // NM
    {PM, PM, PM, PS, ZO, NS, NS}, // NS
    {PM, PM, PS, ZO, NS, NM, NM}, // ZO
    {PS, PS, ZO, NS, NS, NM, NM}, // PS
    {PS, ZO, NS, NM, NM, NM, NB}, // PM
    {ZO, ZO, NM, NM, NM, NB, NB}  // PB
};

// ΔKd 规则表 (关键：在过零点附近增大阻尼抑制振荡)
FuzzySet ruleKd[7][7] = {
    // NB    NM    NS    ZO    PS    PM    PB
    {PS, NS, NB, NB, NB, NM, PS}, // NB
    {PS, NS, NB, NM, NM, NS, ZO}, // NM
    {ZO, NS, NM, NM, NS, NS, ZO}, // NS
    {ZO, NS, NS, PS, PS, PS, ZO}, // ZO (加大正阻尼)
    {ZO, ZO, PS, PS, PM, PM, ZO}, // PS
    {PB, PS, PS, PM, PM, PB, PB}, // PM
    {PB, PM, PM, PM, PS, PB, PB}  // PB
};

// 隶属度计算
inline float trimf(float x, float a, float b, float c)
{
    if (x <= a || x >= c)
        return 0.0f;
    if (x < b)
        return (x - a) / (b - a);
    if (x > b)
        return (c - x) / (c - b);
    return 1.0f;
}

// 反模糊化
float defuzzify(const float strengths[7])
{
    float num = 0.0f, den = 0.0f;
    const float step = 0.1f;
    for (float x = -3.0f; x <= 3.0f; x += step)
    {
        float mu = 0.0f;
        for (int i = 0; i < 7; ++i)
        {
            float m = trimf(x, mf_OUT[i].a, mf_OUT[i].b, mf_OUT[i].c);
            mu = std::max(mu, std::min(strengths[i], m));
        }
        num += x * mu;
        den += mu;
    }
    return (den == 0.0f) ? 0.0f : num / den;
}

// ----------------------------- 模糊PD类 (无积分) -----------------------------
class FuzzyPD
{
public:
    /**
     * @brief 构造函数 (去除了 Ki 相关参数)
     * @param Kp0    基础比例增益 [TUNING] 建议 0.8 ~ 1.5
     * @param Kd0    基础微分增益 [TUNING] 建议 0.3 ~ 0.8
     * @param dt     控制周期 (秒)
     * @param dKp_max ΔKp最大变化范围 [TUNING] 建议 0.3 ~ 0.6
     * @param dKd_max ΔKd最大变化范围 [TUNING] 建议 0.1 ~ 0.3
     */
    FuzzyPD(float Kp0, float Kd0, float dt,
            float dKp_max = 0.5f, float dKd_max = 0.2f)
        : Kp0_(Kp0), Kd0_(Kd0), dt_(dt),
          dKp_scale_(dKp_max / 3.0f),
          dKd_scale_(dKd_max / 3.0f),
          prev_error_(0.0f) {}

    /**
     * @brief 计算控制量
     * @param error     横摇角误差 (度)
     * @param error_dot 横摇角速度 (度/秒)
     * @return 控制输出
     */
    float compute(float error, float error_dot)
    {
        // 1. 输入模糊化
        const float E_SCALE = 0.6f;
        const float EC_SCALE = 0.6f;
        float e_scaled = std::clamp(error * E_SCALE, -6.0f, 6.0f);
        float ec_scaled = std::clamp(error_dot * EC_SCALE, -3.0f, 3.0f);

        float e_mu[7], ec_mu[7];
        for (int i = 0; i < 7; ++i)
        {
            e_mu[i] = trimf(e_scaled, mf_E[i].a, mf_E[i].b, mf_E[i].c);
            ec_mu[i] = trimf(ec_scaled, mf_EC[i].a, mf_EC[i].b, mf_EC[i].c);
        }

        // 2. 规则推理
        float strengthKp[7] = {0.0f}, strengthKd[7] = {0.0f};
        for (int i = 0; i < 7; ++i)
        {
            for (int j = 0; j < 7; ++j)
            {
                float mu = std::min(e_mu[i], ec_mu[j]);
                if (mu == 0.0f)
                    continue;
                int idxKp = static_cast<int>(ruleKp[i][j]);
                int idxKd = static_cast<int>(ruleKd[i][j]);
                strengthKp[idxKp] = std::max(strengthKp[idxKp], mu);
                strengthKd[idxKd] = std::max(strengthKd[idxKd], mu);
            }
        }

        // 3. 反模糊化
        float dKp_norm = defuzzify(strengthKp);
        float dKd_norm = defuzzify(strengthKd);

        // 4. 计算实际增益
        float Kp = Kp0_ + dKp_norm * dKp_scale_;
        float Kd = Kd0_ + dKd_norm * dKd_scale_;
        Kp = std::max(0.0f, Kp);
        Kd = std::max(0.0f, Kd);

        // 5. PD计算 (无积分项)
        float Pout = Kp * error;
        float Dout = Kd * (error - prev_error_) / dt_;
        prev_error_ = error;

        return Pout + Dout;
    }

    void reset() { prev_error_ = 0.0f; }

private:
    float Kp0_, Kd0_, dt_;
    float dKp_scale_, dKd_scale_;
    float prev_error_;
};

// =====================================================================
// 核心控制函数实现
// =====================================================================

// 航速控制类，航速提升（原航速-最优）/辅助制动（原辅助-急停），根据实船数据
class SpeedOptimizer
{
public:
    SpeedOptimizer() = default; // 参数移除，构造函数简化

    // 10kn，27mm航速最高，5mm航速最小
    // 15kn，42mm航速最高，5mm航速最小
    // 20kn，0mm航速最高，50mm航速最小

    // 航速提升（原航速-最优）模式（加速优化）
    std::pair<float, float> compute_optimal_extension(float current_speed,
                                                      float left_current,
                                                      float right_current,
                                                      float max_extension)
    {

        float x = current_speed;
        float y = 0.0f;

        // 半浸桨作用下，难以辅助起滑，因此不执行辅助。
        // if (x <= 5.0f)
        // {
        //     y = 0.0f;
        // }
        // else if (x <= 10.0f)
        // {
        //     // 从(5,0)到(10,27)线性插值
        //     y = 27.0f * (x - 5.0f) / 5.0f;
        // }
        // else if (x <= 15.0f)
        // {
        //     // 从(10,27)到(15,42)线性插值
        //     y = 27.0f + 15.0f * (x - 10.0f) / 5.0f;
        // }
        // else if (x <= 20.0f)
        // {
        //     // 从(15,42)到(20,0)线性插值
        //     y = 42.0f * (20.0f - x) / 5.0f;
        // }
        // else
        // {
        //     y = 0.0f;
        // }

        return apply_constraints(y, left_current, right_current, max_extension);
    }

    // 辅助制动（原辅助-急停）模式（减速优化）
    std::pair<float, float> compute_brake_extension(float current_speed,
                                                    float left_current,
                                                    float right_current,
                                                    float max_extension)
    {

        float x = current_speed;
        float y = 0.0f;

        if (x <= 5.0f)
        {
            y = 0.0f;
        }
        else if (x < 15.0f)
        {
            // 5到15之间线性增长：0→50
            y = 5.0f * (x - 5.0f);
        }
        else
        {
            // 15及以上保持50
            y = max_extension;
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


// 回转改善（原协调-转弯）控制函数 - 基于舵角的大角度转向
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

        right_target = right_current + pid_output;

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

        left_target = left_current + pid_output;

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

// 回转改善（原协调-转弯）控制函数 - 基于横倾角的大角度转向
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

        right_target = right_current + pid_output;
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

        left_target = left_current + pid_output;
        right_target = 0.0f; // 确保右侧收回为0

        AINFO << "左侧PID计算量: " << pid_output;
        AINFO << "左侧目标伸缩量: " << left_target << "mm";
        AINFO << "右侧强制收回为0";
    }

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(left_target, target_extension));
    float new_right = std::max(0.0f, std::min(right_target, target_extension));

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

// 回转改善（原协调-转弯）通用函数：coord_turn_select=1 使用舵角判断，=2 使用横摇/横倾角判断
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
            error_msg = "回转改善（原协调-转弯）模式（舵角判断）需要提供当前舵角";
            AINFO << "回转改善（原协调-转弯）模式（舵角判断）需要提供当前舵角";
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

        // 计算舵角绝对值以决定分支
        std::pair<float, float> result;
        float rudder_magnitude = std::abs(current_rudder);

        // 舵角小于 5° -> 全部收回（轻微转向不处理）
        if (rudder_magnitude < 5.0f)
        {
            AINFO << "舵角(" << current_rudder << "°) < 5°，截流板全部收回";
            result = {0.0f, 0.0f};
        }

        // 舵角在 5°~10° -> 轻微转向，保持当前伸缩量
        else if (rudder_magnitude >= 5.0f && rudder_magnitude <= 10.0f)
        {
            AINFO << "舵角(" << current_rudder << "°) 在5°~10°范围内，保持当前状态";
            result = {input.left_current, input.right_current};
        }

        // 舵角大于 10° -> 进入大角度转向控制逻辑
        else
        {
            AINFO << "舵角(" << current_rudder << "°) > 10°，进入转向控制";
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
            error_msg = "回转改善（原协调-转弯）模式（横摇角判断）需要提供当前横摇角";
            AINFO << "回转改善（原协调-转弯）模式（横摇角判断）需要提供当前横摇角";
            return {new_left, new_right};
        }

        // 读取横倾角并判断幅值
        float current_roll_angle = input.heel_current.value();
        float roll_magnitude = std::abs(current_roll_angle);
        std::pair<float, float> result;

        // 横倾角小于 6° -> 全部收回
        if (roll_magnitude <= 6.0f)
        {
            AINFO << "横倾角(" << current_roll_angle << "°) < 6°，退出回转改善，截流板全部收回";
            result = {0.0f, 0.0f};
        }
        // 横倾角在 6°~8° -> 维持当前状态
        else if (roll_magnitude > 6.0f && roll_magnitude < 8.0f)
        {
            AINFO << "横倾角(" << current_roll_angle << "°) 在范围内，保持当前状态";
            result = {input.left_current, input.right_current};
        }
        // 横倾角大于 8° -> 使用横倾角驱动的大角度转向控制
        else
        {
            AINFO << "横倾角(" << current_roll_angle << "°) > 8°，进入转向控制";
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

// =============================================================================
// 集成示例 (注意：output 需映射到 -1.0 ~ 1.0 的相对量，由后续 K_output 缩放)
// =============================================================================
FuzzyPD fuzzy_pd_heel(1.2f, 0.5f, 0.1f);

// 姿态稳定（原横倾-最优）函数（接收完整输入并包含前置的日志打印与后置1mm保持逻辑，便于复用）
std::pair<float, float> fuzzy_PD_heel_control(const PID_Input &input)
{
    // 使用 input 中的值，调用者需保证已对必要字段进行存在性校验
    float current_speed = input.current_speed.value_or(0.0f);
    AINFO << "=============================已进入姿态稳定（原横倾-最优）程序===================== ";
    AINFO << "当前接收到的速度" << current_speed << "节"
          << "截流板当前阈值" << input.max_extension << "mm";

    float current_angle = input.heel_current.value_or(0.0f); // 当前纵摇角
    float target_angle = input.heel_target.value_or(0.0f);   // 目标纵摇角，默认为0（水平）

    // 静态变量：保存上一时刻的横摇角，用于计算角速度
    static float last_angle = 0.0f;

    float error = current_angle - target_angle;
    float error_dot = (current_angle - last_angle) / dt;
    last_angle = current_angle;

    float output = fuzzy_pd_heel.compute(error, error_dot);

    // [新增]1. 航速自适应增益缩放 (前馈软限幅)
    // 说明：航速越高，截流板效率越高，需主动减小动作幅度防止震荡和埋艏。
    // 参数解释：40.0f 为使用最高航速；0.3f 为高速下允许的最小增益比例。
    float current_speed_kts = input.current_speed.value_or(0.0f);
    float speed_gain = std::clamp(1.0f - (current_speed_kts / 40.0f), 0.3f, 1.0f);
    float scaled_output = output * speed_gain;

    AINFO << "航速自适应增益: " << speed_gain << ", 缩放后输出: " << scaled_output;

    // 将控制量分配到左右截流板
    float adj_left = input.left_current - K_output * scaled_output;
    float adj_right = input.right_current + K_output * scaled_output;

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

// 横倾角PID控制（经典PID版本，舵角逻辑参考PID_turn_large_control）
std::pair<float, float> PID_heel_control(const PID_Input &input, PIDController &pid)
{
    AINFO << "=============================已进入横倾角PID控制程序=====================";

    float current_roll_angle = input.heel_current.value();
    AINFO << "当前横倾角: " << current_roll_angle << "°";

    // 获取航速，如果未提供则默认为0（安全起见，避免负值影响后续计算）
    float current_speed = input.current_speed.value_or(0.0f);

    // 获取横倾角绝对值
    float abs_angle = std::abs(current_roll_angle);

    // ---------- 横倾缩放因子 (圆滑过渡) ----------
    float roll_scale = 0.0f;

    if (abs_angle >= 8.0f && abs_angle <= 14.0f)
    {
        // 8° ~ 14°：完全生效
        roll_scale = 1.0f;
    }
    else if (abs_angle > 14.0f && abs_angle < 16.0f)
    {
        // 14° ~ 16°：线性衰减至零 (可根据需要改用 smoothstep 更平滑)
        roll_scale = 1.0f - (abs_angle - 14.0f) / 2.0f;
    }
    else if (abs_angle >= 6.0f && abs_angle < 8.0f)
    {
        // 6° ~ 8°：从零平滑升至 1.0 (此处采用线性，足够圆滑)
        roll_scale = (abs_angle - 6.0f) / 2.0f;
    }
    // 其他情况 (abs_angle < 6.0f 或 >= 16.0f) 保持为 0.0f

    // ---------- 航速缩放因子 (平滑阶梯变化) ----------
    float speed_scale = 1.0f;

    // 航速节点 (节) 与对应的缩放因子，可按需调整
    const float speed_points[] = {0.0f, 20.0f, 30.0f, 40.0f, 50.0f};
    const float scale_values[] = {1.0f, 1.0f, 0.75f, 0.5f, 0.0f};
    const int num_points = sizeof(speed_points) / sizeof(speed_points[0]);

    float spd = (current_speed > 0.0f) ? current_speed : 0.0f; // 防止负航速

    if (spd <= speed_points[0])
    {
        speed_scale = scale_values[0];
    }
    else if (spd >= speed_points[num_points - 1])
    {
        speed_scale = scale_values[num_points - 1];
    }
    else
    {
        // 在对应区间内线性插值，实现无跳变平滑过渡
        for (int i = 0; i < num_points - 1; ++i)
        {
            if (spd >= speed_points[i] && spd <= speed_points[i + 1])
            {
                float t = (spd - speed_points[i]) / (speed_points[i + 1] - speed_points[i]);
                // 若希望更柔和的过渡，可将 t 替换为 smoothstep：t = t * t * (3.0f - 2.0f * t);
                speed_scale = scale_values[i] + t * (scale_values[i + 1] - scale_values[i]);
                break;
            }
        }
    }

    // ---------- 最终综合缩放因子 ----------
    float total_scale = roll_scale * speed_scale;

    // 动态目标伸缩量：最大伸缩量乘以缩放因子
    float target_extension = input.max_extension * total_scale;
    AINFO << "当前横摇角绝对值: " << abs_angle << "°，伸缩量缩放因子: " << total_scale
          << "，动态目标伸缩量: " << target_extension << "mm";

    // 判断横倾方向（右倾为正，左倾为负）
    bool is_right_roll = current_roll_angle > 0;
    AINFO << "横倾方向: " << (is_right_roll ? "右倾" : "左倾");

    float left_target = 0.0f;
    float right_target = 0.0f;

    if (is_right_roll)
    {
        // 右倾：右侧下放，左侧收回
        float error = target_extension - input.right_current;

        float pid_output = pid.compute(error, dt);

        right_target = input.right_current + pid_output;

        left_target = 0.0f;

        AINFO << "右侧PID计算量: " << pid_output;
        AINFO << "右侧目标伸缩量: " << right_target << "mm";
        AINFO << "左侧强制收回为0";
    }
    else
    {
        // 左倾：左侧下放，右侧收回
        float error = target_extension - input.left_current;

        float pid_output = pid.compute(error, dt);

        left_target = input.left_current + pid_output;

        right_target = 0.0f;

        AINFO << "左侧PID计算量: " << pid_output;
        AINFO << "左侧目标伸缩量: " << left_target << "mm";
        AINFO << "右侧强制收回为0";
    }

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(left_target, target_extension));
    float new_right = std::max(0.0f, std::min(right_target, target_extension));

    // 防抖动处理
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

    AINFO << "横倾PID控制 - 左侧截流板伸缩量: " << new_left << "mm, 右侧截流板伸缩量: " << new_right << "mm";
    return {new_left, new_right};
}

// 强制收回截流板：返回左右两侧收回值（0,0）
static std::pair<float, float> retract_intercepting_plate()
{
    AINFO << "[控制命令] 强制收回截流板，两侧设为0mm";
    return {0.0f, 0.0f};
}


// 急停检测函数：放在 `PID_parameter_transfer` 外部
// 急停检测窗口（秒），用于日志与检测逻辑
static const int detection_window_seconds = 3;
// 规则要求见代码注释：
//---------------------------------
// 左主机急停
//---------------------------------
static bool left_is_emergency_brake(float current_RPM, float current_speed)
{
    using clock = std::chrono::steady_clock;

    // ===================== 【修改1】独立静态变量，杜绝左右主机串扰 =====================
    // 原问题：左右函数共用static变量，一侧急停两侧同步
    // 修改：所有变量添加 left_ 前缀，左主机独立维护状态
    static std::deque<std::pair<clock::time_point, float>> left_rpm_history;
    static bool left_in_emergency = false;
    static clock::time_point left_t0;
    static float left_min_rpm_since_trigger = std::numeric_limits<float>::infinity();

    auto now = clock::now();

    // ---------- 统一更新历史队列（无论是否急停）----------
    left_rpm_history.emplace_back(now, current_RPM);
    while (!left_rpm_history.empty() &&
           std::chrono::duration_cast<std::chrono::seconds>(now - left_rpm_history.front().first).count() > detection_window_seconds)
    {
        left_rpm_history.pop_front();
    }

    // -------------------------------------------------
    // 如果当前处于急停状态：维护状态并检查退出条件
    if (left_in_emergency)
    {
        left_min_rpm_since_trigger = std::min(left_min_rpm_since_trigger, current_RPM);

        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - left_t0).count();

        // 退出条件 A：航速低于5节
        if (current_speed < 5.0f)
        {
            AINFO << "左主机急停退出：航速低于5节";
            left_in_emergency = false;
            left_min_rpm_since_trigger = std::numeric_limits<float>::infinity();
            return false;
        }

        // 退出条件 B：经过20秒
        if (elapsed >= 20.0)
        {
            AINFO << "左急停退出：已超过20秒";
            left_in_emergency = false;
            left_min_rpm_since_trigger = std::numeric_limits<float>::infinity();
            return false;
        }

        // ===================== 【修改3】修复并启用 条件C（防误报核心） =====================
        // 原条件C逻辑正确，但代码错误：return被注释、缩进混乱
        // 修正：补全return、规范缩进、保留原逻辑
        // 作用：触发急停后5秒内，转速从未降到800以下 → 判定为误触发，退出急停
        if (elapsed >= 5.0)
        {
            if (left_min_rpm_since_trigger > 800.0f)
            {
                AINFO << "左急停退出：触发后5秒内转速未降至800RPM以下，判定为误报，退出急停";
                left_in_emergency = false;
                left_min_rpm_since_trigger = std::numeric_limits<float>::infinity();
                return false; // 【关键修复】原代码注释了此行，导致退出无效
            }
        }

        // 仍然满足急停，保持状态
        return true;
    }

    // ---------- 非急停时：判断是否触发急停 ----------
    auto target_time = now - std::chrono::seconds(1);
    float start_rpm = -1.0f;
    clock::time_point start_time;
    bool found_start = false;

    for (const auto &p : left_rpm_history)
    {
        if (p.first <= target_time)
        {
            start_rpm = p.second;
            start_time = p.first;
            found_start = true;
        }
        else
        {
            break;
        }
    }

    // 条件1：至少1秒的历史数据
    if (!found_start)
    {
        AINFO << "左主机条件1不满足，至少1秒的历史数据达不到";
        return false;
    }

    // 条件2：1秒前转速 > 1200 RPM
    if (start_rpm <= 1200.0f)
    {
        AINFO << "左主机条件2不满足，1秒前转速未超过1200 RPM：" << start_rpm;
        return false;
    }

    float end_rpm = left_rpm_history.back().second;

    // ===================== 【修改4】修复时间计算BUG =====================
    // 原问题：错误使用队列头部时间，应该使用找到的起始点时间
    // 修改：用 start_time 计算时间差，保证下降速率计算100%准确
    double total_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();
    if (total_seconds <= 0.0)
    {
        AINFO << "左主机总计秒数total_seconds：" << total_seconds;
        return false;
    }

    // 计算平均下降速率
    double avg_drop_rate = (start_rpm - end_rpm) / total_seconds;

    // 条件3：过去1秒内平均下降速率 > 300 RPM/s
    if (avg_drop_rate <= 300.0)
    {
        AINFO << "左主机条件3不满足，下降速率未达到300 RPM/s：" << avg_drop_rate;
        return false;
    }

    // 满足所有条件 -> 进入急停状态
    left_in_emergency = true;
    left_t0 = now;
    left_min_rpm_since_trigger = current_RPM;

    AINFO << "检测到急停条件，左主机进入急停状态";
    return true;
}

//---------------------------------
// 右主机急停
//---------------------------------
static bool right_is_emergency_brake(float current_RPM, float current_speed)
{
    using clock = std::chrono::steady_clock;

    // ===================== 【修改1】独立静态变量，杜绝左右主机串扰 =====================
    static std::deque<std::pair<clock::time_point, float>> right_rpm_history;
    static bool right_in_emergency = false;
    static clock::time_point right_t0;
    static float right_min_rpm_since_trigger = std::numeric_limits<float>::infinity();

    auto now = clock::now();

    // ---------- 统一更新历史队列（无论是否急停）----------
    right_rpm_history.emplace_back(now, current_RPM);
    while (!right_rpm_history.empty() &&
           std::chrono::duration_cast<std::chrono::seconds>(now - right_rpm_history.front().first).count() > detection_window_seconds)
    {
        right_rpm_history.pop_front();
    }

    // -------------------------------------------------
    // 如果当前处于急停状态：维护状态并检查退出条件
    if (right_in_emergency)
    {
        right_min_rpm_since_trigger = std::min(right_min_rpm_since_trigger, current_RPM);

        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - right_t0).count();

        // 退出条件 A：航速低于5节
        if (current_speed < 5.0f)
        {
            AINFO << "右主机急停退出：航速低于5节";
            right_in_emergency = false;
            right_min_rpm_since_trigger = std::numeric_limits<float>::infinity();
            return false;
        }

        // 退出条件 B：经过20秒
        if (elapsed >= 20.0)
        {
            AINFO << "右急停退出：已超过20秒";
            right_in_emergency = false;
            right_min_rpm_since_trigger = std::numeric_limits<float>::infinity();
            return false;
        }

        // ===================== 【修改3】修复并启用 条件C =====================
        if (elapsed >= 5.0)
        {
            if (right_min_rpm_since_trigger > 800.0f)
            {
                AINFO << "右急停退出：触发后5秒内转速未降至800RPM以下，判定为误报，退出急停";
                right_in_emergency = false;
                right_min_rpm_since_trigger = std::numeric_limits<float>::infinity();
                return false; // 【关键修复】补全return
            }
        }

        // 仍然满足急停，保持状态
        return true;
    }

    // ---------- 非急停时：判断是否触发急停 ----------
    auto target_time = now - std::chrono::seconds(1);
    float start_rpm = -1.0f;
    clock::time_point start_time;
    bool found_start = false;

    for (const auto &p : right_rpm_history)
    {
        if (p.first <= target_time)
        {
            start_rpm = p.second;
            start_time = p.first;
            found_start = true;
        }
        else
        {
            break;
        }
    }

    // 条件1：至少1秒的历史数据
    if (!found_start)
    {
        AINFO << "右主机条件1不满足，至少1秒的历史数据达不到";
        return false;
    }

    // 条件2：1秒前转速 > 1200 RPM
    if (start_rpm <= 1200.0f)
    {
        AINFO << "右主机条件2不满足，1秒前转速未超过1200 RPM：" << start_rpm;
        return false;
    }

    float end_rpm = right_rpm_history.back().second;

    // ===================== 【修改4】修复时间计算BUG =====================
    double total_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();
    if (total_seconds <= 0.0)
    {
        AINFO << "右主机总计秒数total_seconds：" << total_seconds;
        return false;
    }

    // 计算平均下降速率
    double avg_drop_rate = (start_rpm - end_rpm) / total_seconds;

    // 条件3：过去1秒内平均下降速率 > 300 RPM/s
    if (avg_drop_rate <= 300.0)
    {
        AINFO << "右主机条件3不满足，下降速率未达到300 RPM/s：" << avg_drop_rate;
        return false;
    }

    // 满足所有条件 -> 进入急停状态
    right_in_emergency = true;
    right_t0 = now;
    right_min_rpm_since_trigger = current_RPM;

    AINFO << "检测到急停条件，右主机进入急停状态";
    return true;
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

    // ===================== 把这两个变量加在这里 =====================
    static bool lock_left_emergency = false;   // 左侧急停锁存
    static bool lock_right_emergency = false;  // 右侧急停锁存

    // ============ 所有模式通用的数据校验 ============
    // 先使用has_value()检查是否有值，然后再用value()来获取值
    // 航速验证
    if (!input.current_speed.has_value())
    {
        error_msg = "需要提供当前航速";
        AERROR << error_msg; // 使用AERROR宏

        // 构建错误返回结果
        PID_Output output;
        output.new_left = 0;
        output.new_right = 0;
        output.error_code = 10; // 通用错误码，表示速度未提供
        output.error_msg = error_msg;
        return output;
    }

    // 航速赋值
    float current_speed = input.current_speed.value();
    AINFO << "当前航速: " << current_speed << "节";

    if (current_speed < 4)
    {
        error_msg = "航速必须大于4";
        AERROR << error_msg; // 使用AERROR宏

        // 构建错误返回结果
        PID_Output output;
        output.new_left = 0;
        output.new_right = 0;
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
        output.new_left = 0;
        output.new_right = 0;
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
        output.new_left = 0;
        output.new_right = 0;
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
        // 内部我们根据实时数据选择合适的子策略（航速提升（原航速-最优）/辅助制动（原辅助-急停）/回转改善（原协调-转弯）/姿态稳定（原横倾-最优）），
        // 但不改变外部传入的 mode 字段（保持外部可见为自动）。

        // ========== 静态变量用于状态保持 ==========
        // static 关键字在函数作用域声明变量后，该变量在函数多次调用间保持其值不被销毁。
        // 这里用来记住上一次自动选择的逻辑状态，避免每次都清空历史数据或状态。
        static int current_logic_state = 31; // 31:航速提升（原航速-最优）, 32:辅助制动（原辅助-急停）, 33:回转改善（原协调-转弯）, 35:姿态稳定（原横倾-最优）
        static int previous_logic_state = 31; // 上一次的逻辑状态，用于日志对比和状态保持

        // ========== 参数检查（必须字段） ==========
        // has_value() 用于检查 std::optional 是否包含值；若没有值我们不能继续自动判别。
        if (!input.current_left_RPM.has_value() || !input.current_right_RPM.has_value() || !input.current_rudder.has_value())
        {
            // 当必要字段缺失时，设置错误信息并退出当前 case（保留现有 new_left/new_right）
            error_msg = "自动模式需要提供左右主机转速、舵角等必要数据";
            AERROR << error_msg;
            error_code = 101;
            break; // 终止 case 1，返回默认或之前的截流板值
        }

        // 横倾角检查
        if (!input.heel_current.has_value())
        {
            // 当必要字段缺失时，设置错误信息并退出当前 case（保留现有 new_left/new_right）
            error_msg = "自动模式需要提供横倾角等必要数据";
            AERROR << error_msg;
            error_code = 101;
            break; // 终止 case 1，返回默认或之前的截流板值
        }

        // 从 std::optional 安全取值：value() 在 has_value() 为 true 时返回内部值
        float current_left_RPM = input.current_left_RPM.value();   // 左主机转速
        float current_right_RPM = input.current_right_RPM.value(); // 右主机转速
        float current_rudder = input.current_rudder.value();       // 舵角

        float current_heel = input.heel_current.value(); // 横倾角

        // ========== 辅助制动（原辅助-急停）检测（优先级最高） ==========
        // 使用封装函数检测急停（内部维护检测所需的历史队列）

        // 左主机急停判定
        bool left_emergency_brake_detected = left_is_emergency_brake(current_left_RPM, current_speed);

        // 右主机急停判定
        bool right_emergency_brake_detected = right_is_emergency_brake(current_right_RPM, current_speed);

        // ========== 根据实时数据选择逻辑状态 ==========
        // 优先级1: 急停检测（左机或右机任一触发即进入辅助制动模式）
        if (left_emergency_brake_detected || right_emergency_brake_detected)
        {
            current_logic_state = 32; // 32 表示辅助制动（原辅助-急停）
            AINFO << "[自动模式] 检测到急停条件（"
                  << (left_emergency_brake_detected ? "左主机" : "")
                  << (left_emergency_brake_detected && right_emergency_brake_detected ? "、" : "")
                  << (right_emergency_brake_detected ? "右主机" : "")
                  << "触发），使用辅助制动（原辅助-急停）模式";
        }
        else
        {
            // ===================== 【修复点】 =====================
            // 退出急停模式时，必须清空急停锁存，防止输出卡住
            lock_left_emergency = false;
            lock_right_emergency = false;
            // ======================================================

            // 获取绝对值便于判断
            float abs_rudder = std::abs(current_rudder);
            float abs_heel = std::abs(current_heel);

            // -------------------- 优先级2：舵角明确触发转弯 (>10°) --------------------
            if (abs_rudder > 10.0f)
            {
                current_logic_state = 33;
                AINFO << "[自动模式] 舵角" << current_rudder << "°，使用回转改善（原协调-转弯）模式";
            }
            // -------------------- 优先级3：横倾明确触发姿态稳定 (>6°) --------------------
            // 【关键修改】将横倾触发判断提前，使其优先级高于舵角迟滞区间
            else if (abs_heel > 6.0f)
            {
                current_logic_state = 35;
                AINFO << "[自动模式] 横倾角" << current_heel << "°，使用姿态稳定（原横倾-最优）模式";
            }
            // -------------------- 优先级4：迟滞保持逻辑（仅在未触发新条件时保持原状态）--------------------
            else
            {
                // 4.1 转弯迟滞：当前是转弯模式且舵角 ≥ 5°，保持转弯
                if (current_logic_state == 33 && abs_rudder >= 5.0f)
                {
                    // 状态不变，仍为 33
                    AINFO << "[自动模式] 舵角" << current_rudder << "°，处于转弯迟滞区间，保持回转改善模式";
                }
                // 4.2 横倾迟滞：当前是横倾模式且横倾角 ≥ 4°，保持横倾
                else if (current_logic_state == 35 && abs_heel >= 4.0f)
                {
                    // 状态不变，仍为 35
                    AINFO << "[自动模式] 横倾角" << current_heel << "°，处于横倾迟滞区间，保持姿态稳定模式";
                }
                // 4.3 其他情况：回退到航速提升
                else
                {
                    // 【修正】若当前状态是转弯或横倾，但已低于迟滞下限，则强制退出
                    if (current_logic_state == 33 || current_logic_state == 35)
                    {
                        AINFO << "[自动模式] 舵角=" << current_rudder << "°，横倾=" << current_heel
                              << "°，退出" << (current_logic_state == 33 ? "回转改善" : "姿态稳定") << "模式";
                    }
                    current_logic_state = 31;
                    AINFO << "[自动模式] 进入航速提升/航速优先（原航速-最优）模式";
                }
            }
        }

        // ========== 状态切换时复位对应 PID ==========
        if (current_logic_state != previous_logic_state)
        {
            AINFO << "[自动模式] 子状态切换: " << previous_logic_state << " -> " << current_logic_state;
            switch (current_logic_state)
            {
            case 33: // 进入回转改善模式，复位转弯PID
                pid_big_turn.reset();
                AINFO << "[PID复位] 已复位 pid_big_turn";
                break;
            case 35: // 进入姿态稳定模式，复位横倾PID及模糊PD
                pid_heel.reset();
                fuzzy_pd_heel.reset(); // 如有需要也可复位模糊PD
                AINFO << "[PID复位] 已复位 pid_heel 及 fuzzy_pd_heel";
                break;
            // 其他模式（31航速提升、32辅助制动）不使用PID，无需复位
            default:
                break;
            }
            previous_logic_state = current_logic_state;
        }

        // ========== 根据选择的逻辑状态执行相应的控制模式 ==========
        // 直接复制case 31-35的代码，但只在内部使用
        switch (current_logic_state)
        {

        // 航速提升（原航速-最优）模式(同步实现了纵倾最优)
        case 31:
        {
            AINFO << "=============================切换为航速提升（原航速-最优）/纵倾最优模式========================";

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

            AINFO << "[航速提升（原航速-最优）模式] 目标伸缩量: 左=" << new_left // 使用AINFO宏
                  << "mm, 右=" << new_right << "mm";
            break;
        }

        case 32: // 辅助制动（原辅助-急停）模式 (对应原case 32)
        {
            AINFO << "[自动模式-辅助制动（原辅助-急停）模式]";

            AINFO << "当前接收到的速度" << current_speed
                  << "截流板当前阈值" << input.max_extension;

            SpeedOptimizer optimizer;
            auto result = optimizer.compute_brake_extension(
                current_speed,
                input.left_current,
                input.right_current,
                input.max_extension);

            float base_target = result.first; // 左右对称，取任意一侧即可

            // 🔴 核心修复：锁存逻辑（根治二次收回、左右串扰）
            if (left_emergency_brake_detected)
            {
                lock_left_emergency = true;
            }
            if (right_emergency_brake_detected)
            {
                lock_right_emergency = true;
            }

            // 🔴 独立输出：哪侧急停哪侧动
            float target_left = lock_left_emergency ? base_target : 0.0f;
            float target_right = lock_right_emergency ? base_target : 0.0f;

            // 5. 物理范围限制（确保 0 ~ max_extension）
            target_left = std::max(0.0f, std::min(target_left, input.max_extension));
            target_right = std::max(0.0f, std::min(target_right, input.max_extension));

            // 6. 防抖动处理（针对单侧单独处理，避免因微小变化频繁动作）
            const float kDeadZone = 1.0f;
            if (std::abs(target_left - input.left_current) < kDeadZone)
                target_left = input.left_current;
            if (std::abs(target_right - input.right_current) < kDeadZone)
                target_right = input.right_current;

            new_left = target_left;
            new_right = target_right;

            AINFO << "[辅助制动（原辅助-急停）模式] 目标伸缩量: 左=" << new_left
                  << "mm, 右=" << new_right << "mm";
            break;
        }

        case 33: // 回转改善（原协调-转弯）模式 (对应原case 33)
        {
            AINFO << "[自动模式-回转改善（原协调-转弯）模式]";

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

            AINFO << "[自动模式-回转改善（原协调-转弯）] 最终伸缩量: 左=" << new_left << "mm, "
                  << "右=" << new_right << "mm";
            break;
        }

        case 35: // 姿态稳定（原横倾-最优）模式 (对应原case 35)
        {
            if (!input.heel_current.has_value())
            {
                error_code = 350;
                error_msg = "姿态稳定（原横倾-最优）模式需要提供当前横倾/摇角度";
                AERROR << error_msg; // 使用AERROR宏
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

    // 航速提升（原航速-最优）模式(同步实现了纵倾最优)
    case 31:
    {
        AINFO << "=============================切换为航速提升（原航速-最优）/纵倾最优模式========================";

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

        AINFO << "[航速提升（原航速-最优）模式] 目标伸缩量: 左=" << new_left // 使用AINFO宏
              << "mm, 右=" << new_right << "mm";
        break;
    }

    // 辅助制动（原辅助-急停）模式
    case 32:
    {
        AINFO << "=============================切换为辅助制动（原辅助-急停）模式========================";

        // 先判断是否有主机转速数据
        if (!input.current_left_RPM.has_value() || !input.current_right_RPM.has_value())
        {
            error_code = 321;
            error_msg = "辅助制动模式需要提供主机转速（current_left_RPM 和 current_right_RPM）";
            AERROR << error_msg;

            // 需求：无数据 → 退出模式，输出0
            new_left = 0.0f;
            new_right = 0.0f;
            break;
        }
        else
        {
            AINFO << "当前左主机转速：" << input.current_left_RPM.value();
            AINFO << "当前右主机转速：" << input.current_right_RPM.value();
        }

        float current_left_RPM = input.current_left_RPM.value();   // 左主机转速
        float current_right_RPM = input.current_right_RPM.value(); // 右主机转速

        // 左主机急停判定
        bool left_emergency_brake_detected = left_is_emergency_brake(current_left_RPM, current_speed);

        // 右主机急停判定
        bool right_emergency_brake_detected = right_is_emergency_brake(current_right_RPM, current_speed);

        // ===================== 核心修复1：锁存变量移到函数顶部，此处增加复位逻辑 =====================
        // 作用：急停完全退出时，自动清零锁存，截流板收回
        if (!left_emergency_brake_detected)
        {
            lock_left_emergency = false;
        }
        if (!right_emergency_brake_detected)
        {
            lock_right_emergency = false;
        }

        // ===================== 核心修复2：无急停 → 直接输出0，退出模式（满足你的需求） =====================
        if (!left_emergency_brake_detected && !right_emergency_brake_detected)
        {
            error_code = 322;
            error_msg = "辅助制动（急停）判定未通过：左、右主机均未检测到急停条件，输出0";
            AINFO << error_msg;
            
            // 严格按照需求：退出模式 → 伸缩量置0
            new_left = 0.0f;
            new_right = 0.0f;
            break;
        }

        AINFO << "左主机急停状态: " << (left_emergency_brake_detected ? "触发" : "未触发");
        AINFO << "右主机急停状态: " << (right_emergency_brake_detected ? "触发" : "未触发");

        AINFO << "=============================切换为辅助制动（原辅助-急停）模式========================";

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
        float base_target = result.first; // 左右对称，取任意一侧即可

        // 🔴 核心修复：锁存逻辑（根治二次收回、左右串扰）
        if (left_emergency_brake_detected)
        {
            lock_left_emergency = true;
        }
        if (right_emergency_brake_detected)
        {
            lock_right_emergency = true;
        }

        // 🔴 独立输出：哪侧急停哪侧动
        float target_left = lock_left_emergency ? base_target : 0.0f;
        float target_right = lock_right_emergency ? base_target : 0.0f;

        // 5. 物理范围限制（确保 0 ~ max_extension）
        target_left = std::max(0.0f, std::min(target_left, input.max_extension));
        target_right = std::max(0.0f, std::min(target_right, input.max_extension));

        // 6. 防抖动处理（针对单侧单独处理，避免因微小变化频繁动作）
        const float kDeadZone = 1.0f;
        if (std::abs(target_left - input.left_current) < kDeadZone)
            target_left = input.left_current;
        if (std::abs(target_right - input.right_current) < kDeadZone)
            target_right = input.right_current;

        new_left = target_left;
        new_right = target_right;

        AINFO << "[辅助制动（原辅助-急停）模式] 目标伸缩量: 左=" << new_left
              << "mm, 右=" << new_right << "mm";
        break;
    }

    // 回转改善（原协调-转弯）模式
    case 33:
    {
        // 修改此变量的值以切换判断模式：1 使用舵角判断，2 使用横摇角判断
        int coord_turn_select = 1; // <-- 修改为 1 或 2 即可切换（仅修改此参数，无需改动头文件）

        // 回转改善（原协调-转弯）通用函数调用，result接收主要返回值，error_code/error_msg通过引用即可被修改
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

        AINFO << "[回转改善（原协调-转弯）] 最终伸缩量: 左=" << new_left << "mm, "
              << "右=" << new_right << "mm";
        break;
    }

    // 纵倾最优控制（暂时空置）
    case 34:
    {
        AINFO << "[暂时空置] （该功能合并至航速提升（原航速-最优）模式）"; // 使用AINFO宏
        break;
    }

    // 姿态稳定（原横倾-最优）模式 备份
    case 35:
    {
        if (!input.heel_current.has_value())
        {
            error_code = 350;
            error_msg = "姿态稳定（原横倾-最优）模式需要提供当前横倾/摇角度";
            AERROR << error_msg;
            break;
        }

        float current_heel = input.heel_current.value();

        // 横倾角绝对值 > 6°：调用 PID_heel_control 计算伸缩量
        if (fabs(current_heel) > 6.0f)
        {
            AINFO << "[姿态稳定] 横倾角 " << current_heel << "°，进入姿态稳定模式（横倾最优模式）";
            auto result = PID_heel_control(input, pid_heel);
            new_left = result.first;
            new_right = result.second;
        }
        // 横倾角绝对值在 [4, 6] 区间内：保持伸缩量不变（不修改 new_left/new_right）
        else if (fabs(current_heel) >= 4.0f && fabs(current_heel) <= 6.0f)
        {
            AINFO << "[姿态稳定] 横倾角 " << current_heel << "°，处于迟滞区间，保持当前伸缩量不变";
            // new_left 和 new_right 保持原有值，不执行 PID 控制
        }
        // 横倾角绝对值 < 4°：打印提示并退出当前 case
        else
        {
            AINFO << "[姿态稳定] 横倾角 " << current_heel << "° < 4°，不符合姿态稳定条件，退出";
            new_left = 0.0f;
            new_right = 0.0f;
            break;
        }

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
