/**
 * @file:    alg_processor.cpp
 * @brief:   PID算法调度数据处理：包含数据输入、数据处理、数据输出
 *
 *
 * @author:  Wesley
 * @date:    2025-07-21 10:09:25
 * @note:
 */

#include "alg_processor.h"

#include <glog/logging.h>
#include <log.h>

// 构造函数：初始化 stopFlag
AlgProcessor::AlgProcessor() : stop_flag_(false), save_data_(false), thread_pool_(1)
{
  if (save_data_)
  {
    file_name_ = get_now_string("%Y-%m-%d_%H-%M-%S") + ".csv";
    AINFO << "filename: " << file_name_;
    std::ofstream file(file_name_);
    if (!file.is_open())
    {
      AERROR << "Failed to open file " << file_name_;
      return;
    }

    // 写表头
    file << "time,mode,currend_speed,pitch_current,pitch_target,heel_current,heel_target,left_current,right_current,max_extension,current_heading,current_rudder,new_left,new_right,error_code\n";
    file.close();
  }
}

// 析构函数：确保停止处理并清理资源
AlgProcessor::~AlgProcessor()
{
  stop(); // 确保停止所有处理
}

// 设置回调函数
void AlgProcessor::set_callback(Callback callback)
{
  std::lock_guard<std::mutex> lock(mtx_);
  callback_ = callback;
}

// 添加数据到输入队列
void AlgProcessor::add_data(const AlgInput &data)
{
  AWARN << "添加数据=============================";
  std::lock_guard<std::mutex> lock(mtx_);
  input_queue_.push(data);

  AWARN << "完成添加数据==========================";
  cv_input_.notify_one(); // 通知等待线程有数据可处理
}

// 数据处理函数
void AlgProcessor::process_data()
{
  AERROR << "进入process====================";
  while (!stop_flag_)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    // 阻塞等待数据
    cv_input_.wait(lock, [this]()
                   { return !input_queue_.empty() || stop_flag_; }); // 等待直到输入队列有数据或停止标志

    if (stop_flag_ && input_queue_.empty())
    {
      return; // 如果停止标志被设置且队列为空，退出
    }
    // AERROR<<"AAAAAAAAAAAAAAAAAAAA====================";
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // 数据非空
    if (!input_queue_.empty())
    {
      // 获取第一条数据
      AlgInput alg_input_data = input_queue_.front();
      input_queue_.pop();
      lock.unlock(); // 释放锁，允许其他线程操作队列

      //-----------------------------调用PID算法部分-----------------------------
      // 调用核心函数
      // 获取开始时间点
      auto start = std::chrono::steady_clock::now();
      AERROR << "调用PID算法==================================";
      AlgResult alg_result = PID_parameter_transfer(alg_input_data);
      AWARN << "调用算法结果--" << "左截流板: " << alg_result.new_left << "mm, "
            << "右截流板: " << alg_result.new_right << "mm";

      // 获取结束时间点
      auto end = std::chrono::steady_clock::now();
      // 计算时间间隔（以毫秒为单位）
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      // 输出时间间隔
      AWARN <<alg_input_data.mode<< "控制算法调用单次耗时: " << duration_ms.count() << " 毫秒";
      //-----------------------------调用PID算法部分-----------------------------

      //-----------------------------执行回调函数-----------------------------
      // 回调函数
      if (callback_)
      {
        callback_(alg_result);
      }

      if (save_data_)
      {
        thread_pool_.enqueue(&AlgProcessor::save_data, this, alg_input_data, alg_result);
      }

      //-----------------------------执行回调函数-----------------------------
    }
  }
}

// 停止处理
void AlgProcessor::stop()
{
  stop_flag_ = true;
  cv_input_.notify_all(); // 唤醒所有线程
}

// 清除数据
void AlgProcessor::clear()
{
  std::lock_guard<std::mutex> lock(mtx_);
  while (!input_queue_.empty())
  {
    input_queue_.pop();
  }
}

void AlgProcessor::save_data(AlgInput in, AlgResult res)
{
  int mode = in.mode;
  float current_speed = in.current_speed.has_value() ? in.current_speed.value() : -1;         // 当前船舶航速（单位：节）
  float pitch_current = in.pitch_current.has_value() ? in.pitch_current.value() : -361;       // 当前纵倾角度
  float pitch_target = in.pitch_target.has_value() ? in.pitch_target.value() : -361;          // 目标纵倾角度
  float heel_current = in.heel_current.has_value() ? in.heel_current.value() : -361;          // 当前横倾角度
  float heel_target = in.heel_target.has_value() ? in.heel_target.value() : -361;             // 目标横倾角度
  float left_current = in.left_current;                                                       // 左侧截流板当前伸缩量（单位：毫米）
  float right_current = in.right_current;                                                     // 右侧截流板当前伸缩量（单位：毫米）
  float max_extension = in.max_extension;                                                     // 截流板最大允许伸缩量（单位：毫米）
  float current_heading = in.current_heading.has_value() ? in.current_heading.value() : -361; // 船舶当前艏向
  float current_rudder = in.current_rudder.has_value() ? in.current_rudder.value() : -361;

  float new_left = res.new_left;   // 更新后的左侧伸缩量
  float new_right = res.new_right; // 更新后的右侧伸缩量
  int error_code = res.error_code; // 错误处理字段

  std::string time = get_now_string();

  std::ofstream file(file_name_, std::ios::app); // 追加模式
  if (!file.is_open())
  {
    AERROR << "无法打开文件！" << file_name_;
    return;
  }
  file << "'" << time << "'" << "," << mode << "," << current_speed << "," << pitch_current << ","
       << pitch_target << "," << heel_current << "," << heel_target << "," << left_current << ","
       << right_current << "," << max_extension << "," << current_heading << "," << current_rudder << ","
       << new_left << "," << new_right << "," << error_code << "\n";
  file.close();
}

std::string AlgProcessor::get_now_string(std::string format)
{
  // 获取当前时间点
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);

  // 转换为本地时间
  std::tm tm = *std::localtime(&t);

  // 取毫秒部分
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  // 格式化到字符串
  std::ostringstream oss;
  oss << std::put_time(&tm, format.c_str());
  oss << '.' << std::setfill('0') << std::setw(3) << ms.count(); // 毫秒，补齐3位
  return oss.str();
}
