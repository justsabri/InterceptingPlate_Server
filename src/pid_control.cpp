
#include <glog/logging.h>
#include <sys/stat.h> // 添加此头文件，用于创建目录

#include "log.h"  // 确保包含log.h头文件
#include "pid_control.h"

#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>

#include <cfloat>  // 添加FLT_MAX的定义
#include <vector>
#include <tuple>

// #include <logging.h>

// 全局常量定义
const float K_output = 3.0f;  // 输出增益系数
const float dt = 0.1f;        // 时间间隔（单位：秒）

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

  float compute(float error, float dt) {
    integral += error * dt;                                       // 积分项累加
    float derivative = (error - prev_error) / dt;                 // 微分项计算
    float output = Kp * error + Ki * integral + Kd * derivative;  // PID输出
    prev_error = error;  // 更新上一次误差
    return output;
  }

 private:
  const float Kp;    // 比例系数
  const float Ki;    // 积分系数
  const float Kd;    // 微分系数
  float prev_error;  // 上一次误差
  float integral;    // 积分累积值
};

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
// 全局PID控制器实例
// =====================================================================

// 纵倾控制全局参数
const float Kp_pitch = 0.6f;
const float Ki_pitch = 0.02f;
const float Kd_pitch = 0.1f;
PIDController pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch);

// 横倾控制全局参数
const float Kp_heel = 0.5f;
const float Ki_heel = 0.01f;
const float Kd_heel = 0.05f;
PIDController pid_heel(Kp_heel, Ki_heel, Kd_heel);

// 速度优先控制全局参数
const float Kp_speed = 0.5f;
const float Ki_speed = 0.01f;
const float Kd_speed = 0.1f;
AdaptivePIDSpeed pid_speed(Kp_speed, Ki_speed, Kd_speed, 2.0f, 0.1f);

// 协调转弯控制全局参数
//小角度
const float Kp_small_turn = 0.4f;
const float Ki_small_turn = 0.01f;
const float Kd_small_turn = 0.1f;
PIDController pid_small_turn(Kp_small_turn, Ki_small_turn, Kd_small_turn);

//大角度
const float Kp_big_turn = 0.8f;
const float Ki_big_turn = 0.01f;
const float Kd_big_turn = 0.1f;
PIDController pid_big_turn(Kp_big_turn, Ki_big_turn, Kd_big_turn);



// =====================================================================
// 仿真函数
// =====================================================================

/**
 * @brief 根据航速和等效截流板伸缩量计算船舶角度
 *
 * C++实现说明：
 * 1. 使用map替代Python字典，C++17支持类内初始化
 * 2. 使用结构体数组存储多项式系数
 */
float simulation_angle(float speed, float x) {
  // 定义多项式系数结构体
  struct SpeedCoeff {
    int speed;
    float a, b, c, d, e;
  };

  // 速度与系数映射表
  const SpeedCoeff coefficients[] = {
      {15, -2E-07f, 2E-05f, -0.0008f, 0.0385f, -4.46f},
      {25, -4E-07f, 5E-05f, -0.0026f, 0.1199f, -4.86f},
      {35, -2E-06f, 0.0002f, -0.0058f, 0.1466f, -4.84f},
      {45, -3E-06f, 0.0003f, -0.0085f, 0.1493f, -3.61f}};

  // 查找最接近的速度系数
  const SpeedCoeff* coeff_ptr = nullptr;
  float min_diff = 100.0f;

  for (const auto& coeff : coefficients) {
    float diff = std::abs(speed - coeff.speed);
    if (diff < min_diff) {
      min_diff = diff;
      coeff_ptr = &coeff;
    }
  }

  if (!coeff_ptr) {
    return 0.0f;  // 默认值
  }

  // 计算船舶角度
  return coeff_ptr->a * std::pow(x, 4) + coeff_ptr->b * std::pow(x, 3) +
         coeff_ptr->c * std::pow(x, 2) + coeff_ptr->d * x + coeff_ptr->e;
}

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
//

//纵倾最优函数
std::pair<float, float> PID_pitch_control(
    float current_angle, float target_angle,
    float left_current, float right_current,
    float max_extension, PIDController& pid)
{
    AINFO << "=============================已进入纵倾最优程序===================== ";
    AINFO << "当前纵倾角度" << current_angle;
    AINFO << "目标纵倾角度" << target_angle;

    // 使用静态变量保存状态信息，用于在多次调用间保持记忆
    static int state = 0;                      // 状态机：0=全局搜索，1=局部微调
    static float best_angle = FLT_MAX;         // 历史最优纵倾角绝对值
    static float best_extension = 0.0f;        // 历史最优伸缩量
    static int search_direction = 1;           // 搜索方向：1=增加，-1=减少
    static float last_angle = 0.0f;            // 上一次的纵倾角

    // 检查纵倾角是否发生较大变化（超过2°），若是则触发全局搜索
    if (fabs(current_angle - last_angle) > 2.0f) {
        state = 0; // 重置为全局搜索状态
        best_angle = FLT_MAX; // 重置最优值
        AINFO << "纵倾角变化超过2°，触发全局搜索";
    }
    last_angle = current_angle; // 更新上一次纵倾角

    float new_left, new_right;

    if (state == 0) {
        // 全局搜索模式：遍历所有可能的伸缩量寻找全局最优
        AINFO << "全局搜索模式";

        // 计算当前伸缩量对应的纵倾角绝对值
        float abs_angle = fabs(current_angle);

        // 更新历史最优值
        if (abs_angle < best_angle) {
            best_angle = abs_angle;
            best_extension = left_current; // 左右对称，取左侧值即可
            AINFO << "更新历史最优值，角度:" << best_angle << ", 伸缩量:" << best_extension;
        }

        // 确定下一步搜索方向
        if (left_current >= max_extension) {
            search_direction = -1; // 达到上限，开始减少
            AINFO << "达到最大伸缩量，开始减少搜索";
        }
        else if (left_current <= 0.0f) {
            search_direction = 1; // 达到下限，开始增加
            AINFO << "达到最小伸缩量，开始增加搜索";
        }

        // 计算搜索步长（最大伸缩量的5%）
        float search_step = max_extension * 0.05f;

        // 应用搜索步长
        new_left = left_current + search_direction * search_step;
        new_right = right_current + search_direction * search_step;

        // 检查是否完成全局搜索（遍历完所有可能值）
        if (search_direction == -1 && new_left <= 0.0f) {
            // 已完成全局搜索，切换到局部微调模式
            state = 1;
            new_left = best_extension;
            new_right = best_extension;
            AINFO << "全局搜索完成，切换到局部微调模式，最优伸缩量:" << best_extension;
        }
    }
    else {
        // 局部微调模式：在历史最优值附近微调
        AINFO << "局部微调模式，当前最优伸缩量:" << best_extension;

        // 使用PID进行微调控制
        float error = target_angle - current_angle;
        float output = pid.compute(error, dt);

        // 应用微调
        new_left = best_extension + K_output * output;
        new_right = best_extension + K_output * output;
    }

    // 应用物理限制
    new_left = std::max(0.0f, std::min(new_left, max_extension));
    new_right = std::max(0.0f, std::min(new_right, max_extension));

    return { new_left, new_right };
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
        // 检查必要的输入数据
        if (!input.current_rudder.has_value()) {
            error_code = 330;
            error_msg = "协调转弯模式需要提供当前舵角";
            AERROR << error_msg;
            break;
        }

        // 获取当前舵角（有符号值，正右负左）
        float current_rudder = input.current_rudder.value();
        AINFO << "当前舵角: " << current_rudder << "°";

        // 根据舵角大小选择控制策略
        float rudder_magnitude = std::abs(current_rudder);
        std::pair<float, float> result;

        if (rudder_magnitude >= 5.0f) {
            // 大角度转向控制
            result = PID_turn_large_control(current_rudder,
                input.left_current,
                input.right_current,
                input.max_extension,
                pid_big_turn);
        }
        else {
            // 小角度转向控制
            result = PID_turn_small_control(current_rudder,
                input.left_current,
                input.right_current,
                input.max_extension,
                pid_small_turn);
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


     // 纵倾/摇最优模式
    case 34: 
     { 
      if (!input.pitch_current.has_value()) 
      {
        error_code = 340;
        error_msg = "纵倾模式需要提供当前纵倾角度";
        AERROR << error_msg;  // 使用AERROR宏
        break;
      }

      float current_angle = input.pitch_current.value();
      float target_angle = input.pitch_target.value_or(-1.0f);

      AINFO << "当前纵倾角度: " << current_angle << "°";  // 使用AINFO宏

      auto result = PID_pitch_control(current_angle, target_angle,
                                      input.left_current, input.right_current,
                                      input.max_extension, pid_pitch);

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


      AINFO << "[纵倾模式] 目标伸缩量: 左=" << new_left  // 使用AINFO宏
            << "mm, 右=" << new_right << "mm";
      break;
    }



    case 35: {  // 横倾控制模式
      if (!input.heel_current.has_value()) {
        error_code = 350;
        error_msg = "横倾模式需要提供当前横倾角度";
        AERROR << error_msg;  // 使用AERROR宏
        break;
      }

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
