#pragma once
#include <optional>
#include <string> // 添加string头文件

// 输入参数结构体
struct PID_Input {
  int mode;  // 控制模式（1=舒适，2=节能，31=航速优先，32=辅助急停，33=协调转弯，34=纵倾最优，35=横倾最优）
  std::optional<float> current_speed;    // 当前船舶航速（单位：节）
  std::optional<float> pitch_current;    // 当前纵倾角度
  std::optional<float> pitch_target;     // 目标纵倾角度
  std::optional<float> heel_current;     // 当前横倾角度
  std::optional<float> heel_target;      // 目标横倾角度
  float left_current;                    // 左侧截流板当前伸缩量（单位：毫米）
  float right_current;                   // 右侧截流板当前伸缩量（单位：毫米）
  float max_extension;                   // 截流板最大允许伸缩量（单位：毫米）
  std::optional<float> current_heading;  // 船舶当前艏向
  std::optional<float> current_rudder;   // 船舶当前舵角
};

// 输出参数结构体（对应Python返回的元组）
struct PID_Output {

  int mode;   // 控制模式（1=舒适，2=节能，31=航速优先，32=辅助急停，33=协调转弯，34=纵倾最优，35=横倾最优）
  float new_left;   // 更新后的左侧伸缩量
  float new_right;  // 更新后的右侧伸缩量

  // 新增错误处理字段
  int error_code = 0;          // 0=成功, 非0=错误，模式+“0”=缺少相关数据，模式+“1”=数据存在问题
  std::string error_msg;       // 错误描述
};

// 声明主接口函数
PID_Output PID_parameter_transfer(const PID_Input& input);