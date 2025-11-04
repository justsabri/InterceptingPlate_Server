
#include <glog/logging.h>
#include <sys/stat.h> // 添加此头文件，用于创建目录

#include "log.h"  // 确保包含log.h头文件
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
const float K_output = 3.0f;  // 输出增益系数
const float dt = 0.5f;        // 时间间隔（单位：秒）

// =====================================================================
// PID控制器类实现
// =====================================================================

/**
 * @brief PID控制器类，实现基本的PID控制算法
 */
class PIDController {
 public:
  PIDController(float Kp, float Ki, float Kd)
      : Kp(Kp), Ki(Ki), Kd(Kd), prev_error(0), integral(0) {}


  // 添加reset函数
   void reset() 
   {
   integral = 0.0f;
   prev_error = 0.0f;
   }

    float compute(float error, float dt) {
        // 积分项（抗饱和）
        if (std::abs(error) < 5.0f) {
            integral += error * dt;
        } else {
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
  const float Kp;    // 比例系数
  const float Ki;    // 积分系数
  const float Kd;    // 微分系数
  float prev_error = 0.0f;  // 上一次误差
  float integral = 0.0f;    // 积分累积值
};

// =====================================================================
// 改进的PID控制器类
// =====================================================================

class ImprovedPIDController {
 public:
  ImprovedPIDController(float Kp, float Ki, float Kd, float max_integral = 10.0f, float dead_zone = 0.5f)
      : Kp(Kp), Ki(Ki), Kd(Kd), max_integral(max_integral), dead_zone(dead_zone), 
        prev_error(0), integral(0) {}

  void reset() {
    integral = 0.0f;
    prev_error = 0.0f;
  }

  float compute(float error, float dt) {
    // 死区处理 - 小误差时不响应
    if (std::abs(error) < dead_zone) {
      return 0.0f;
    }
    
    // 积分项（抗饱和）
    if (std::abs(error) < 5.0f) {
      integral += error * dt;
      // 积分限幅
      integral = std::clamp(integral, -max_integral, max_integral);
    } else {
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
  const float Kp;          // 比例系数
  const float Ki;          // 积分系数
  const float Kd;          // 微分系数
  const float max_integral; // 积分上限
  const float dead_zone;   // 死区大小
  float prev_error = 0.0f; // 上一次误差
  float integral = 0.0f;   // 积分累积值
};

// =====================================================================
// 全局PID控制器实例
// =====================================================================

// // 纵倾控制全局参数
// const float Kp_pitch = 0.4f;
// const float Ki_pitch = 0.01f;
// const float Kd_pitch = 0.05f;
// PIDController pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch);

// 横倾控制全局参数
const float Kp_heel = 0.5f;
const float Ki_heel = 0.01f;
const float Kd_heel = 0.05f;
PIDController pid_heel(Kp_heel, Ki_heel, Kd_heel);

// 速度优先控制全局参数
const float Kp_speed = 0.5f;
const float Ki_speed = 0.01f;
const float Kd_speed = 0.1f;
// AdaptivePIDSpeed pid_speed(Kp_speed, Ki_speed, Kd_speed, 2.0f, 0.1f);

// 协调转弯控制全局参数
//小角度
const float Kp_small_turn = 0.4f;
const float Ki_small_turn = 0.01f;
const float Kd_small_turn = 0.1f;
PIDController pid_small_turn(Kp_small_turn, Ki_small_turn, Kd_small_turn);

//大角度
const float Kp_big_turn = 0.4f;
const float Ki_big_turn = 0.01f;
const float Kd_big_turn = 0.1f;
PIDController pid_big_turn(Kp_big_turn, Ki_big_turn, Kd_big_turn);





// =====================================================================
// 自适应PID速度控制器类实现
// =====================================================================

/**
 * @brief 自适应PID速度控制器，支持速度变化方向感知
 */
class AdaptivePIDSpeed {
 public:
  AdaptivePIDSpeed(float Kp, float Ki, float Kd, float min_step = 1.0f,
                   float speed_threshold = 0.1f)
      : Kp(Kp),
        Ki(Ki),
        Kd(Kd),
        min_step(min_step),
        speed_threshold(speed_threshold),
        previous_speed(std::nullopt),
        last_error(0),
        integral(0),
        last_output(0),
        direction(1) {}

  float compute(float current_speed, float dt) {
    // 首次调用初始化
    if (!previous_speed.has_value()) {
      previous_speed = current_speed;
      return 0.0f;
    }

    // 计算航速变化量（误差）
    float speed_change = current_speed - previous_speed.value();
    float error = speed_change;

    // 忽略微小变化（抗噪声）
    if (std::abs(speed_change) < speed_threshold) {
      error = 0.0f;
    }

    // PID计算
    integral += error * dt;
    float derivative = (dt > 0) ? (error - last_error) / dt : 0.0f;
    float output = Kp * error + Ki * integral + Kd * derivative;

    // 动态调整方向：若速度下降则反转方向
    if (speed_change < -speed_threshold) {
      direction *= -1;
      integral = 0.0f;  // 重置积分项
    }

    // 应用方向并限制最小步长
    float adjusted_output = output * direction;
    if (std::abs(adjusted_output) < min_step) {
      adjusted_output = (adjusted_output >= 0) ? min_step : -min_step;
    }

    // 更新状态
    last_error = error;
    previous_speed = current_speed;
    last_output = adjusted_output;

    return adjusted_output;
  }

 private:
  const float Kp;
  const float Ki;
  const float Kd;
  const float min_step;
  const float speed_threshold;
  std::optional<float> previous_speed;  // 上一次航速
  float last_error;                     // 上一次误差
  float integral;                       // 积分累积值
  float last_output;                    // 上一次输出
  int direction;                        // 调整方向：1=增加，-1=减少
};

// 航速最优控制类，根据实船数据
class SpeedOptimizer {
public:
    SpeedOptimizer() = default;  // 参数移除，构造函数简化

    std::pair<float, float> compute_extension(float current_speed,
        float left_current,
        float right_current,
        float max_extension) 
        
        {
        float x = current_speed;
        float y = 0.0f;  // 最佳伸缩量

        // 分段计算最佳伸缩量
        if (x < 5) {
            y = 0;
        }
        else if (x < 8.7) {
            y = 10.81f * x - 54.05f;
        }
        else if (x < 17) {
            y = 1.205f * x + 29.52f;
        }
        else if (x <= 50) {
            y = 0.052f * x * x - 4.93f * x + 118;
        }
        else {
            y = 0;
        }

        // 物理范围限制
        float new_ext = std::max(0.0f, std::min(y, max_extension));

        // 获取当前较小的伸缩量
        float current_min_ext =
            (left_current < right_current) ? left_current : right_current;

        // 微小变化处理：差值小于1时保持当前值
        float final_ext = new_ext;
        if (std::abs(new_ext - current_min_ext) < 1.0f) {
            final_ext = current_min_ext;
        }

        return { final_ext, final_ext };
    }
};


// =====================================================================
// 核心控制函数实现
// =====================================================================

// 航速最优（速度PID自适应）函数
std::pair<float, float> adaptive_PID_speed_control(float current_speed,
    float left_current,
    float right_current,
    float max_extension,
    AdaptivePIDSpeed& pid)
{
    // 计算调整量（两侧同步）
    float adjustment = pid.compute(current_speed, dt);

    // 同步更新两侧伸缩量
    float new_left = left_current + K_output * adjustment;
    float new_right = right_current + K_output * adjustment;

    // 应用物理限制
    new_left = std::max(0.0f, std::min(new_left, max_extension));
    new_right = std::max(0.0f, std::min(new_right, max_extension));

    return { new_left, new_right };
}



// 协调转弯控制函数 - 基于横倾角的大角度转向
std::pair<float, float> PID_turn_large_control_by_roll(float current_roll_angle,
    float left_current,
    float right_current,
    float max_extension,
    PIDController& pid) {

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

    if (is_right_roll) {
        // 右倾：右侧截流板下放，左侧收回为0
        float error = target_extension - right_current;
        float pid_output = pid.compute(error, dt);

        right_target = right_current + error;
        left_target = 0.0f; // 确保左侧收回为0

        AINFO << "右侧PID计算量: " << pid_output;
        AINFO << "右侧目标伸缩量: " << right_target << "mm";
        AINFO << "左侧强制收回为0";
    }
    else {
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

    return { new_left, new_right };
}



// 协调转弯控制函数 - 大角度转向（5°及以上）
std::pair<float, float> PID_turn_large_control(float current_rudder,
    float left_current,
    float right_current,
    float max_extension,
    PIDController& pid) {
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

    return { new_left, new_right };
}

// 协调转弯控制函数 - 小角度转向（5°以下）
std::pair<float, float> PID_turn_small_control(float current_rudder,
    float left_current,
    float right_current,
    float max_extension,
    PIDController& pid) 
{
    AINFO << "=============================已进入小角度转向控制程序=====================";
    AINFO << "当前舵角: " << current_rudder << "°";

    // 确定转向方向
    bool is_right_turn = current_rudder > 0;
    float rudder_magnitude = std::abs(current_rudder);

    AINFO << "转向方向: " << (is_right_turn ? "右转" : "左转");
    AINFO << "舵角绝对值: " << rudder_magnitude << "°";

    // 小角度转向：线性比例计算 (每1°舵角对应10mm伸缩量)
    float target_extension = rudder_magnitude * 10.0f;
    AINFO << "小角度转向，计算伸缩量: " << target_extension << "mm";

    // 确保不超过最大限制
    target_extension = std::min(target_extension, max_extension);
    AINFO << "限制后伸缩量: " << target_extension << "mm";

    // 根据转向方向设置控制策略
    float left_target = 0.0f;
    float right_target = 0.0f;

    if (is_right_turn) {
        // 右转：右侧截流板下放，左侧收回为0
        float error = target_extension - right_current;
        float pid_output = pid.compute(error, dt);
        right_target = right_current + pid_output;
        left_target = 0.0f; // 确保左侧收回为0
        //AINFO << "右侧PID计算量: " << pid_output;
        AINFO << "右侧目标伸缩量: " << right_target << "mm";
        AINFO << "左侧强制收回为0";
    }
    else {
        // 左转：左侧截流板下放，右侧收回为0
        float error = target_extension - left_current;
        float pid_output = pid.compute(error, dt);
        left_target = left_current + pid_output;
        right_target = 0.0f; // 确保右侧收回为0
        //AINFO << "左侧PID计算量: " << pid_output;
        AINFO << "左侧目标伸缩量: " << left_target << "mm";
        AINFO << "右侧强制收回为0";
    }

    // 应用物理限制
    float new_left = std::max(0.0f, std::min(left_target, max_extension));
    float new_right = std::max(0.0f, std::min(right_target, max_extension));

    AINFO << "小角度转向控制完成 - 左侧截流板伸缩量: " << new_left << "mm, 右侧截流板伸缩量: " << new_right << "mm";

    return { new_left, new_right };
}


////纵倾最优函数
//std::pair<float, float> PID_pitch_control(
//   float current_angle, float target_angle, 
//   float left_current, float right_current,
//   float max_extension, PIDController& pid) 
//{
//
// AINFO << "=============================已进入纵倾最优程序===================== " ;
// AINFO << "当前纵倾角度" << current_angle;
// AINFO << "目标纵倾角度" << target_angle;
//
//
// float error = target_angle - current_angle;  // 计算误差
// float output = pid.compute(error, dt);       // PID计算
//
// // 同步调整两侧伸缩量
// float new_left = left_current + K_output * output;
// float new_right = right_current + K_output * output;
//
// // 应用物理限制
// new_left = std::max(0.0f, std::min(new_left, max_extension));
// new_right = std::max(0.0f, std::min(new_right, max_extension));
//
// return {new_left, new_right};
//}

// 全局静态变量用于记录状态
static int last_mode = 0;
static std::vector<float> history_angles;
static std::vector<float> history_extensions;
static int history_count = 0;
static float optimal_error = std::numeric_limits<float>::max();
static float optimal_angle = 0.0f;
static float optimal_extension = 0.0f;
static bool switched_to_optimal = false;
static int stabilization_counter = 0;
static const int STABILIZATION_THRESHOLD = 20; // 稳定计数阈值

// 改进的纵倾最优函数
std::pair<float, float> improved_PID_pitch_control(
    float current_angle, float target_angle, 
    float left_current, float right_current,
    float max_extension, ImprovedPIDController& pid) 
{
    // 模式切换时重置状态
    if (last_mode != 34) {
        history_angles.clear();
        history_extensions.clear();
        history_count = 0;
        optimal_error = std::numeric_limits<float>::max();
        optimal_angle = 0.0f;
        optimal_extension = 0.0f;
        switched_to_optimal = false;
        stabilization_counter = 0;
        pid.reset();
        last_mode = 34;
        AINFO << "模式切换，重置所有状态";
    }

    // 更新历史数据
    history_angles.push_back(current_angle);
    float current_extension = (left_current + right_current) / 2;
    history_extensions.push_back(current_extension);
    
    if (history_angles.size() > 10) {
        history_angles.erase(history_angles.begin());
        history_extensions.erase(history_extensions.begin());
    }
    history_count = history_angles.size();

    // 计算平均纵倾角
    float average_current = 0.0f;
    if (history_count > 0) {
        average_current = std::accumulate(history_angles.begin(), history_angles.end(), 0.0f) / history_count;
    }
    
    float current_angle_error = std::abs(average_current - target_angle);

    // 更新最优值
    if (current_angle_error < optimal_error) {
        optimal_error = current_angle_error;
        optimal_angle = average_current;
        optimal_extension = current_extension;
        stabilization_counter = 0; // 重置稳定计数器
        AINFO << "更新最优值 - 误差: " << optimal_error 
              << ", 角度: " << optimal_angle 
              << ", 伸缩量: " << optimal_extension;
    } else {
        // 增加稳定计数器
        stabilization_counter++;
        AINFO << "稳定计数器: " << stabilization_counter;
    }

    // 检查是否需要切换控制目标
    if (!switched_to_optimal && 
        (current_angle_error > optimal_error + 2.0f || stabilization_counter >= STABILIZATION_THRESHOLD)) {
        switched_to_optimal = true;
        AINFO << "切换控制目标到最优角度: " << optimal_angle;
    }

    // 确定控制目标
    float control_target = switched_to_optimal ? optimal_angle : target_angle;
    float error = control_target - current_angle;
    
    // 计算PID输出
    float output = pid.compute(error, dt); 

    // 计算新的伸缩量
    float new_extension = current_extension + output;
    new_extension = std::clamp(new_extension, 0.0f, max_extension);

    // 如果已经切换到最优控制目标且接近最优值，保持稳定
    if (switched_to_optimal && std::abs(current_extension - optimal_extension) < 1.0f) {
        new_extension = optimal_extension;
        AINFO << "保持最优伸缩量: " << optimal_extension;
    }

    AINFO << "控制目标: " << control_target 
          << ", 当前角度: " << current_angle
          << ", 误差: " << error
          << ", PID输出: " << output
          << ", 新伸缩量: " << new_extension;

    return {new_extension, new_extension};
}





    // 横倾最优函数
    std::pair<float, float> PID_heel_control(float current_angle,
                                            float target_angle, float left_current,
                                            float right_current,
                                            float max_extension,
                                            PIDController& pid)
    {
    float error = current_angle - target_angle;  // 计算误差
    float output = pid.compute(error, dt);       // PID计算

    AINFO << "=============================已进入横倾最优程序===================== " ;
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

PID_Output PID_parameter_transfer(const PID_Input& input) {
  // 使用log.h封装的宏替代原生glog
  AINFO << "接收到的截流板最大伸缩量: " << input.max_extension << "mm";
  AINFO << "左侧当前: " << input.left_current
        << "mm, 右侧当前: " << input.right_current << "mm";

  float new_left = input.left_current;
  float new_right = input.right_current;
  int error_code = 0; // 0=成功，非0=错误
  std::string error_msg;

  switch (input.mode) {

    case 1: { // 舒适优先模式
      AINFO << "[舒适模式] 保持当前状态";  // 使用AINFO宏
      break;

    }

   

    case 2: {  // 速度优先模式(节能模式)
      AERROR << "=============================切换为节能模式";
      if (!input.current_speed.has_value()) {
        error_code = 20;
        error_msg = "速度模式需要提供当前航速";
        AERROR << error_msg;  // 使用AERROR宏
        break;
      }

      float current_speed = input.current_speed.value();
      if (current_speed < 0) {
        error_code = 21;
        error_msg = "航速必须大于0";
        AERROR << error_msg;  // 使用AERROR宏
        break;
      }
      AERROR<<"当前接收到的速度"<<current_speed<<"截流板当前阈值"<<input.max_extension;
      SpeedOptimizer optimizer;
      auto result = optimizer.compute_extension(
          current_speed, input.left_current, input.right_current,
          input.max_extension);

      new_left = result.first;
      new_right = result.second;
      AINFO << "[速度模式] 目标伸缩量: 左=" << new_left  // 使用AINFO宏
            << "mm, 右=" << new_right << "mm";
      break;
    }



    case 31: {  // 速度优先模式(节能模式)
      AERROR << "=============================切换为速度优先模式========================";
      if (!input.current_speed.has_value()) {
        error_code = 310;
        error_msg = "速度模式需要提供当前航速";
        AERROR << error_msg;  // 使用AERROR宏
        break;
      }

      float current_speed = input.current_speed.value();
      if (current_speed < 0) {
        error_code = 311;
        error_msg = "航速必须大于0";
        AERROR << error_msg;  // 使用AERROR宏
        break;
      }

      
      AERROR<<"当前接收到的速度"<<current_speed<<"截流板当前阈值"<<input.max_extension;
      
      SpeedOptimizer optimizer;
      auto result = optimizer.compute_extension(
          current_speed, input.left_current, input.right_current,
          input.max_extension);

      new_left = result.first;
      new_right = result.second;
      AINFO << "[速度模式] 目标伸缩量: 左=" << new_left  // 使用AINFO宏
            << "mm, 右=" << new_right << "mm";
      break;
    }


    case 32: { // 辅助急停模式
      AINFO << "[辅助急停模式] 保持当前状态";  // 使用AINFO宏
      break;

    }


    // 协调转弯控制模式
    case 33: 
    {  
        // // 检查必要的输入数据
        // if (!input.current_rudder.has_value()) {
        //     error_code = 330;
        //     error_msg = "协调转弯模式需要提供当前舵角";
        //     AERROR << error_msg;
        //     break;
        // }




        float current_speed = input.current_speed.value();
        AERROR<<"当前接收到的速度"<<current_speed<<"截流板当前阈值"<<input.max_extension;

        //// 根据舵角大小选择控制策略
        //// 获取当前舵角（有符号值，正右负左）
        //// float current_rudder = input.current_rudder.value();//接真实数据时，用该代码

        ////虚拟数据，舵角
        //float current_rudder = -3;

        //AINFO << "当前舵角: " << current_rudder << "°";
 
        //float rudder_magnitude = std::abs(current_rudder);
        //std::pair<float, float> result;

        //if (rudder_magnitude >= 5.0f) {
        //    // 大角度转向控制
        //    result = PID_turn_large_control(current_rudder,
        //        input.left_current,
        //        input.right_current,
        //        input.max_extension,
        //        pid_big_turn);
        //}
        //else {
        //    // 小角度转向控制
        //    result = PID_turn_small_control(current_rudder,
        //        input.left_current,
        //        input.right_current,
        //        input.max_extension,
        //        pid_small_turn);
        //}

        // 获取当前横倾角
        float current_roll_angle = input.heel_current.value();
        float roll_magnitude = std::abs(current_roll_angle);
        std::pair<float, float> result;

        // 根据横倾角大小选择控制策略
        if (roll_magnitude < 4.0f) {
            // 横倾角小于4°：全部收回
            AINFO << "横倾角(" << current_roll_angle << "°) < 4°，截流板全部收回";
            result = { 0.0f, 0.0f };
        }
        else if (roll_magnitude >= 4.0f && roll_magnitude <= 6.0f) {
            // 横倾角在4°-6°范围内：保持当前状态
            AINFO << "横倾角(" << current_roll_angle << "°) 在4°-6°范围内，保持当前状态";
            result = { input.left_current, input.right_current };
        }
        else {
            // 横倾角大于6°：进行大角度转向控制
            AINFO << "横倾角(" << current_roll_angle << "°) > 6°，进入大角度转向控制";
            result = PID_turn_large_control_by_roll(current_roll_angle,
                input.left_current,
                input.right_current,
                input.max_extension,
                pid_big_turn);
        }



        new_left = result.first;
        new_right = result.second;

        // 检查变化量是否小于1mm（满足条件则保持当前值）
        if (std::abs(new_left - input.left_current) < 1.0f) {
            AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << input.left_current << "mm";
            new_left = input.left_current;
        }

        if (std::abs(new_right - input.right_current) < 1.0f) {
            AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << input.right_current << "mm";
            new_right = input.right_current;
        }

        AINFO << "[协调转弯] 最终伸缩量: 左=" << new_left << "mm, 右=" << new_right << "mm";
        break;
    }

    //纵倾最优控制
    case 34:
{
    if (!input.pitch_current.has_value()) {
        error_code = 340;
        error_msg = "纵倾模式需要提供当前纵倾角度";
        break;
    }

    float current_angle = input.pitch_current.value();
    float target_angle = input.pitch_target.value_or(-1.0f);

    // 使用改进的PID控制器
    static ImprovedPIDController improved_pid_pitch(0.3f, 0.005f, 0.03f, 5.0f, 0.3f);
    
    auto result = improved_PID_pitch_control(current_angle, target_angle,
                                            input.left_current, input.right_current,
                                            input.max_extension, improved_pid_pitch);
    
    new_left = result.first;
    new_right = result.second;

    // 检查变化量
    if (std::abs(new_left - input.left_current) < 1.0f) {
        new_left = input.left_current;
    }
    if (std::abs(new_right - input.right_current) < 1.0f) {
        new_right = input.right_current;
    }

    AINFO << "[纵倾模式] 目标伸缩量: 左=" << new_left
          << "mm, 右=" << new_right << "mm";
    break;
}
  
    



// 横倾控制模式 备份
    case 35: 
    {  
     if (!input.heel_current.has_value()) {
       error_code = 350;
       error_msg = "横倾模式需要提供当前横倾角度";
       AERROR << error_msg;  // 使用AERROR宏
       break;
     }
     float current_speed = input.current_speed.value();
     AERROR<<"当前接收到的速度"<<current_speed<<"截流板当前阈值"<<input.max_extension;

     float current_angle = input.heel_current.value();
     float target_angle = input.heel_target.value_or(0.0f);

     AINFO << "当前横倾角度: " << current_angle << "°";  // 使用AINFO宏

     auto result = PID_heel_control(current_angle, target_angle,
                                    input.left_current, input.right_current,
                                    input.max_extension, pid_heel);

     new_left = result.first;
     new_right = result.second;


     // 检查变化量是否小于1mm（满足条件则保持当前值）
     if (std::abs(new_left - input.left_current) < 1.0f) {
         AINFO << "左侧伸缩量变化小于1mm，保持当前值: " << input.left_current << "mm";
         new_left = input.left_current;
     }

     if (std::abs(new_right - input.right_current) < 1.0f) {
         AINFO << "右侧伸缩量变化小于1mm，保持当前值: " << input.right_current << "mm";
         new_right = input.right_current;
     }

     AINFO << "[横倾模式] 目标伸缩量: 左=" << new_left  // 使用AINFO宏
           << "mm, 右=" << new_right << "mm";
     break;
    }



    default:
      error_code = 106;
      error_msg = "无效控制模式: " + std::to_string(input.mode);
      AERROR << error_msg;  // 使用AERROR宏
  }

  AINFO << "-------------------------------------------------------";  // 使用AINFO宏

  // 修改返回语句
  PID_Output result;
  result.mode = input.mode;
  result.new_left = new_left;
  result.new_right = new_right;
  result.error_code = error_code;
  result.error_msg = error_msg;
  return result;
}
